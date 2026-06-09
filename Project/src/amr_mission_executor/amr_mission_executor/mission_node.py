#!/usr/bin/env python3
"""
Mission executor node.

Hosts the /execute_mission ActionServer (ExecuteMission interface) and
translates each mission goal into a sequence of Nav2 NavigateToPose
action calls.

Threading: MultiThreadedExecutor + ReentrantCallbackGroup. Execute
callback awaits futures by POLLING with time.sleep() rather than
calling rclpy.spin_once recursively. This is essential: spinning the
same node from within a callback the executor is already driving
causes subtle deadlocks, missed cancel callbacks, and stuck-after-
first-mission behavior.
"""

from __future__ import annotations

import math
import threading
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


SOURCE_FOR_MISSION_TYPE: Dict[str, str] = {
    "grocery": "supermarket",
    "food":    "restaurant",
    "fire":    "fire_station",
    "medical": "pharmacy",
}

PHASE_DISPATCHING              = "dispatching"
PHASE_NAV_TO_SOURCE            = "navigating_to_source"
PHASE_AT_SOURCE                = "at_source"
PHASE_NAV_TO_DESTINATION       = "navigating_to_destination"
PHASE_AT_DESTINATION           = "at_destination"
PHASE_RETURNING_TO_DOCK        = "returning_to_dock"
PHASE_COMPLETE                 = "complete"

ARRIVAL_DWELL_SEC = 0.5    # short dwell between legs (was 1.5)
NAV_GOAL_ACCEPT_TIMEOUT_SEC = 5.0
POLL_INTERVAL_SEC = 0.05   # 20 Hz poll on result future


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class MissionExecutor(Node):
    def __init__(self) -> None:
        super().__init__("mission_executor")

        # ---- Parameters: landmarks ----
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

        # ---- One reentrant callback group shared by client + server ----
        self._cb_group = ReentrantCallbackGroup()

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self._cb_group,
        )

        self._action_server = ActionServer(
            self,
            ExecuteMission,
            "/execute_mission",
            execute_callback=self._execute_mission,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=self._cb_group,
        )

        # State shared between cancel callback and execute callback.
        self._active_nav_handle: Optional[ClientGoalHandle] = None
        self._state_lock = threading.Lock()  # protects _active_nav_handle
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
        """Runs on a different executor thread than execute_callback."""
        self.get_logger().info("Cancel requested by client.")
        with self._state_lock:
            handle = self._active_nav_handle
        if handle is not None:
            self.get_logger().info("Forwarding cancel to active Nav2 goal.")
            handle.cancel_goal_async()
        else:
            self.get_logger().info(
                "No active Nav2 goal to cancel (between legs or pre-flight)."
            )
        return CancelResponse.ACCEPT

    # ---------------- Mission execution ----------------

    def _execute_mission(self, goal_handle: ServerGoalHandle):
        start = time.monotonic()
        request = goal_handle.request

        source_name = SOURCE_FOR_MISSION_TYPE[request.mission_type]
        destination_name = request.destination_house

        legs = [
            (PHASE_NAV_TO_SOURCE,      source_name,        PHASE_AT_SOURCE),
            (PHASE_NAV_TO_DESTINATION, destination_name,   PHASE_AT_DESTINATION),
            (PHASE_RETURNING_TO_DOCK,  "docking_station",  PHASE_COMPLETE),
        ]

        self._publish_feedback(goal_handle, PHASE_DISPATCHING, 0.0)
        time.sleep(0.2)

        # Wait for Nav2 to be available.
        if not self._nav_client.wait_for_server(timeout_sec=NAV_GOAL_ACCEPT_TIMEOUT_SEC):
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

            self._publish_feedback(goal_handle, arrival_phase, 0.0)
            if arrival_phase != PHASE_COMPLETE:
                time.sleep(ARRIVAL_DWELL_SEC)

        # Success path
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

    # ---------------- Nav2 leg — proper future awaiting ----------------

    def _drive_to(
        self,
        mission_handle: ServerGoalHandle,
        target_pose: PoseStamped,
        phase_name: str,
    ) -> Tuple[bool, str]:
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = target_pose

        self._latest_nav_distance_m = 0.0

        # ---- Send goal (don't spin_once; just poll the future) ----
        send_future = self._nav_client.send_goal_async(
            nav_goal,
            feedback_callback=lambda fb: self._on_nav_feedback(
                fb, mission_handle, phase_name),
        )

        elapsed = 0.0
        while not send_future.done() and elapsed < NAV_GOAL_ACCEPT_TIMEOUT_SEC:
            if not rclpy.ok():
                return False, "rclpy shut down during send"
            time.sleep(POLL_INTERVAL_SEC)
            elapsed += POLL_INTERVAL_SEC

        if not send_future.done():
            return False, "send_goal timed out (Nav2 unreachable?)"

        nav_handle: ClientGoalHandle = send_future.result()
        if nav_handle is None or not nav_handle.accepted:
            return False, "Nav2 rejected the goal"

        with self._state_lock:
            self._active_nav_handle = nav_handle

        # ---- Wait for result; poll for cancel ----
        result_future = nav_handle.get_result_async()
        cancel_forwarded = False

        while rclpy.ok() and not result_future.done():
            if mission_handle.is_cancel_requested and not cancel_forwarded:
                self.get_logger().info(
                    f"[execute_loop] cancel detected during {phase_name}, "
                    "forwarding to Nav2"
                )
                nav_handle.cancel_goal_async()
                cancel_forwarded = True
            time.sleep(POLL_INTERVAL_SEC)

        with self._state_lock:
            self._active_nav_handle = None

        if not result_future.done():
            return False, "rclpy shut down during result wait"

        wrapper = result_future.result()
        status = wrapper.status
        if status == 4:    # SUCCEEDED
            return True, ""
        if status == 5:    # CANCELED
            return False, "cancelled"
        return False, f"Nav2 returned status {status}"

    def _on_nav_feedback(self, fb_msg, mission_handle, phase_name) -> None:
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
        elapsed = time.monotonic() - start
        sec_part = int(elapsed)
        result.mission_duration.sec = sec_part
        result.mission_duration.nanosec = int((elapsed - sec_part) * 1_000_000_000)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
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
