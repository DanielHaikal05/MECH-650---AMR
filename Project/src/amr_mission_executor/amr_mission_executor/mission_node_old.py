#!/usr/bin/env python3
"""
Mission executor node.

Hosts the /execute_mission ActionServer (ExecuteMission interface) and
translates each mission goal into a sequence of Nav2 NavigateToPose
action calls.

Mission flow per leg:
  1. Resolve target landmark name to (x, y, yaw) from the landmarks
     parameter dict.
  2. Send a NavigateToPose goal to Nav2.
  3. While Nav2 is navigating, forward feedback to the ExecuteMission
     client (the GUI), translating Nav2's distance_remaining into
     ExecuteMission feedback's distance_to_current_goal.
  4. On Nav2 result, advance to the next leg.

Phases (ExecuteMission.Feedback.current_phase):
  dispatching -> navigating_to_source -> at_source ->
  navigating_to_destination -> at_destination ->
  returning_to_dock -> complete

Cancel handling: if the GUI cancels, we cancel the active Nav2 goal,
wait for it to settle, then return a 'cancelled' result.

Threading: a MultiThreadedExecutor with a ReentrantCallbackGroup is
required because the action-server execute callback awaits futures
from an action client. Without reentrancy, the executor deadlocks
(server callback blocks the same thread the client needs).
"""

from __future__ import annotations

import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import ClientGoalHandle
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from amr_mission_manager.action import ExecuteMission


# Mission type -> source landmark name. Mirrors amr_gui/mission_console.py.
SOURCE_FOR_MISSION_TYPE: Dict[str, str] = {
    "grocery": "supermarket",
    "food":    "restaurant",
    "fire":    "fire_station",
    "medical": "pharmacy",
}

# Phase strings. Must match exactly what the GUI checks against.
PHASE_DISPATCHING              = "dispatching"
PHASE_NAV_TO_SOURCE            = "navigating_to_source"
PHASE_AT_SOURCE                = "at_source"
PHASE_NAV_TO_DESTINATION       = "navigating_to_destination"
PHASE_AT_DESTINATION           = "at_destination"
PHASE_RETURNING_TO_DOCK        = "returning_to_dock"
PHASE_COMPLETE                 = "complete"

ARRIVAL_DWELL_SEC = 1.5  # how long to "wait" at source/destination
NAV_GOAL_TIMEOUT_SEC = 5.0  # how long to wait for Nav2 to accept the goal


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Returns (x, y, z, w) for a rotation about the Z axis only."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class MissionExecutor(Node):
    def __init__(self) -> None:
        super().__init__("mission_executor")

        # ---- Parameters: landmarks dict ----
        # We declare landmark names individually as nested parameters
        # because rclpy doesn't accept dict-of-dicts directly. Each
        # landmark is declared as three floats: <name>.x, <name>.y, <name>.yaw.
        # The launch file populates these from a YAML file.
        self.declare_parameter("landmark_names", [""])
        landmark_names = self.get_parameter("landmark_names").value
        if not landmark_names or landmark_names == [""]:
            self.get_logger().error(
                "No landmarks declared. Pass 'landmark_names' parameter."
            )
        self._landmarks: Dict[str, Tuple[float, float, float]] = {}
        for name in landmark_names:
            if not name:
                continue
            self.declare_parameter(f"{name}.x", 0.0)
            self.declare_parameter(f"{name}.y", 0.0)
            self.declare_parameter(f"{name}.yaw", 0.0)
            x = self.get_parameter(f"{name}.x").value
            y = self.get_parameter(f"{name}.y").value
            yaw = self.get_parameter(f"{name}.yaw").value
            self._landmarks[name] = (float(x), float(y), float(yaw))
            self.get_logger().info(
                f"Loaded landmark '{name}' -> ({x:.2f}, {y:.2f}, {yaw:.2f})"
            )

        self.declare_parameter("global_frame", "map")
        self._global_frame = self.get_parameter("global_frame").value

        # ---- Reentrant callback group (mandatory for nested actions) ----
        self._cb_group = ReentrantCallbackGroup()

        # ---- Nav2 action client ----
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self._cb_group,
        )

        # ---- ExecuteMission action server ----
        self._action_server = ActionServer(
            self,
            ExecuteMission,
            "/execute_mission",
            execute_callback=self._execute_mission,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=self._cb_group,
        )

        # Track the active Nav2 goal handle so we can forward cancel.
        self._active_nav_handle: Optional[ClientGoalHandle] = None

        # Latest Nav2 feedback distance (forwarded to GUI feedback).
        self._latest_nav_distance_m: float = 0.0

        self.get_logger().info(
            "Mission executor ready. Action: /execute_mission. "
            f"Loaded {len(self._landmarks)} landmark(s)."
        )

    # ---------------- Goal lifecycle ----------------

    def _on_goal_request(self, goal_request) -> GoalResponse:
        mission_type = goal_request.mission_type
        destination = goal_request.destination_house

        if mission_type not in SOURCE_FOR_MISSION_TYPE:
            self.get_logger().warn(
                f"Rejecting goal: unknown mission_type '{mission_type}'"
            )
            return GoalResponse.REJECT

        source = SOURCE_FOR_MISSION_TYPE[mission_type]
        for required in (source, destination, "docking_station"):
            if required not in self._landmarks:
                self.get_logger().warn(
                    f"Rejecting goal: landmark '{required}' is not configured."
                )
                return GoalResponse.REJECT

        self.get_logger().info(
            f"Accepting mission: {mission_type} (src={source}) -> {destination}"
        )
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel requested by client.")
        # Forward cancel to Nav2 if a leg is in flight.
        if self._active_nav_handle is not None:
            self.get_logger().info("Forwarding cancel to active Nav2 goal.")
            self._active_nav_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    # ---------------- Mission execution ----------------

    def _execute_mission(self, goal_handle: ServerGoalHandle):
        start = self.get_clock().now()
        request = goal_handle.request

        source_name = SOURCE_FOR_MISSION_TYPE[request.mission_type]
        destination_name = request.destination_house

        legs = [
            (PHASE_NAV_TO_SOURCE,      source_name,        PHASE_AT_SOURCE),
            (PHASE_NAV_TO_DESTINATION, destination_name,   PHASE_AT_DESTINATION),
            (PHASE_RETURNING_TO_DOCK,  "docking_station",  PHASE_COMPLETE),
        ]

        # Phase: dispatching
        self._publish_feedback(goal_handle, PHASE_DISPATCHING, 0.0)
        time.sleep(0.3)

        # Wait for Nav2 to come up.
        if not self._nav_client.wait_for_server(timeout_sec=NAV_GOAL_TIMEOUT_SEC):
            return self._fail(
                goal_handle, start,
                "Nav2 NavigateToPose action server is not available.",
            )

        for nav_phase, landmark_name, arrival_phase in legs:
            if goal_handle.is_cancel_requested:
                return self._cancelled(goal_handle, start)

            target_pose = self._landmark_to_pose_stamped(landmark_name)
            self.get_logger().info(
                f"Phase {nav_phase}: navigating to {landmark_name} "
                f"({target_pose.pose.position.x:.2f}, "
                f"{target_pose.pose.position.y:.2f})"
            )

            ok, reason = self._drive_to(goal_handle, target_pose, nav_phase)
            if goal_handle.is_cancel_requested:
                return self._cancelled(goal_handle, start)
            if not ok:
                return self._fail(
                    goal_handle, start,
                    f"Navigation to {landmark_name} failed: {reason}",
                )

            # Arrived at this leg.
            self._publish_feedback(goal_handle, arrival_phase, 0.0)
            if arrival_phase != PHASE_COMPLETE:
                time.sleep(ARRIVAL_DWELL_SEC)

        # All legs complete.
        goal_handle.succeed()
        result = ExecuteMission.Result()
        result.success = True
        result.message = (
            f"Mission complete: {request.mission_type} delivered to "
            f"{destination_name}."
        )
        self._set_duration(result, start)
        self.get_logger().info(result.message)
        return result

    # ---------------- Nav2 leg ----------------

    def _drive_to(
        self,
        mission_handle: ServerGoalHandle,
        target_pose: PoseStamped,
        phase_name: str,
    ) -> Tuple[bool, str]:
        """Drives to one pose. Returns (success, reason_if_failed)."""

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = target_pose

        self._latest_nav_distance_m = 0.0
        self._current_phase_for_feedback = phase_name

        send_future = self._nav_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda fb: self._on_nav_feedback(
                fb, mission_handle, phase_name),
        )

        # Wait synchronously inside the execute callback. Reentrant
        # callback group lets the executor drive the future to completion.
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=NAV_GOAL_TIMEOUT_SEC)
        if not send_future.done():
            return False, "send_goal future timed out"

        nav_handle: ClientGoalHandle = send_future.result()
        if not nav_handle.accepted:
            return False, "Nav2 rejected the goal"

        self._active_nav_handle = nav_handle

        result_future = nav_handle.get_result_async()
        # Spin until result OR cancel.
        while rclpy.ok() and not result_future.done():
            if mission_handle.is_cancel_requested:
                # Cancel forwarded by _on_cancel_request, but be defensive.
                nav_handle.cancel_goal_async()
            rclpy.spin_once(self, timeout_sec=0.1)

        self._active_nav_handle = None

        if not result_future.done():
            return False, "result future never completed"

        wrapper = result_future.result()
        # status: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        status = wrapper.status
        if status == 4:
            return True, ""
        if status == 5:
            return False, "cancelled"
        return False, f"Nav2 returned status {status}"

    def _on_nav_feedback(self, fb_msg, mission_handle, phase_name) -> None:
        """Forward Nav2 feedback into ExecuteMission feedback."""
        nav_fb = fb_msg.feedback
        self._latest_nav_distance_m = float(nav_fb.distance_remaining)
        self._publish_feedback(
            mission_handle, phase_name, self._latest_nav_distance_m
        )

    # ---------------- Helpers ----------------

    def _landmark_to_pose_stamped(self, name: str) -> PoseStamped:
        x, y, yaw = self._landmarks[name]
        ps = PoseStamped()
        ps.header.frame_id = self._global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        return ps

    def _publish_feedback(
        self, goal_handle: ServerGoalHandle, phase: str, distance_m: float
    ) -> None:
        fb = ExecuteMission.Feedback()
        fb.current_phase = phase
        fb.distance_to_current_goal = float(distance_m)
        fb.current_robot_pose = PoseStamped()
        fb.current_robot_pose.header.frame_id = self._global_frame
        fb.current_robot_pose.header.stamp = self.get_clock().now().to_msg()
        goal_handle.publish_feedback(fb)

    def _cancelled(self, goal_handle: ServerGoalHandle, start) -> ExecuteMission.Result:
        goal_handle.canceled()
        result = ExecuteMission.Result()
        result.success = False
        result.message = "Mission cancelled by user."
        self._set_duration(result, start)
        return result

    def _fail(self, goal_handle: ServerGoalHandle, start, message: str) -> ExecuteMission.Result:
        goal_handle.abort()
        result = ExecuteMission.Result()
        result.success = False
        result.message = message
        self._set_duration(result, start)
        self.get_logger().error(message)
        return result

    def _set_duration(self, result: ExecuteMission.Result, start) -> None:
        elapsed = self.get_clock().now() - start
        ns = elapsed.nanoseconds
        result.mission_duration.sec = int(ns // 1_000_000_000)
        result.mission_duration.nanosec = int(ns % 1_000_000_000)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
