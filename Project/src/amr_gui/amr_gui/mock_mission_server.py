#!/usr/bin/env python3
"""
Mock ExecuteMission action server for testing the GUI without the real
mission manager.

Phase 9 introduced this module. NOT part of the production system —
it exists solely to validate the GUI ↔ action contract before Phase 8
implements the real server.

Behavior:
  - Accepts every goal.
  - Walks through the standard phase sequence (dispatching →
    navigating_to_source → at_source → navigating_to_destination →
    at_destination → returning_to_dock → complete) with 2-second
    delays between phases.
  - Decreases distance_to_current_goal linearly during navigating_* phases.
  - Reports success.
  - Honors cancel: if cancelled mid-mission, returns success=false
    with message="cancelled by user".

Usage:
    ros2 run amr_gui mock_mission_server
"""

from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from amr_mission_manager.action import ExecuteMission


PHASE_SEQUENCE = [
    ("dispatching", 1.0, 0.0),
    ("navigating_to_source", 4.0, 10.0),
    ("at_source", 1.0, 0.0),
    ("navigating_to_destination", 4.0, 8.0),
    ("at_destination", 1.0, 0.0),
    ("returning_to_dock", 4.0, 12.0),
    ("complete", 0.0, 0.0),
]
# Each entry: (phase_name, duration_seconds, initial_distance_m)
# Total nominal mission: ~15 seconds.

FEEDBACK_RATE_HZ = 2.0  # 2 feedback messages per second


class MockMissionServer(Node):
    def __init__(self) -> None:
        super().__init__("mock_mission_server")
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ExecuteMission,
            "/execute_mission",
            execute_callback=self._execute,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=self._cb_group,
        )
        self.get_logger().info("Mock mission server ready on /execute_mission")

    def _on_goal_request(self, goal_request) -> GoalResponse:
        self.get_logger().info(
            f"Goal received: type={goal_request.mission_type}, "
            f"dest={goal_request.destination_house}"
        )
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing mock mission")
        start_time = self.get_clock().now()
        feedback = ExecuteMission.Feedback()
        feedback.current_robot_pose = PoseStamped()
        feedback.current_robot_pose.header.frame_id = "map"

        for phase_name, duration, initial_distance in PHASE_SEQUENCE:
            if goal_handle.is_cancel_requested:
                return self._make_cancelled_result(goal_handle, start_time)

            tick_count = max(1, int(duration * FEEDBACK_RATE_HZ))
            for tick in range(tick_count):
                if goal_handle.is_cancel_requested:
                    return self._make_cancelled_result(goal_handle, start_time)

                progress = tick / max(1, tick_count - 1) if tick_count > 1 else 1.0
                feedback.current_phase = phase_name
                feedback.distance_to_current_goal = max(
                    0.0, initial_distance * (1.0 - progress)
                )
                feedback.current_robot_pose.header.stamp = (
                    self.get_clock().now().to_msg()
                )
                goal_handle.publish_feedback(feedback)
                time.sleep(1.0 / FEEDBACK_RATE_HZ)

            if phase_name == "complete":
                # Final feedback already sent above
                pass

        # Mission complete
        goal_handle.succeed()
        result = ExecuteMission.Result()
        result.success = True
        result.message = "mock mission completed successfully"
        elapsed = self.get_clock().now() - start_time
        result.mission_duration.sec = int(elapsed.nanoseconds // 1_000_000_000)
        result.mission_duration.nanosec = int(
            elapsed.nanoseconds % 1_000_000_000
        )
        return result

    def _make_cancelled_result(self, goal_handle: ServerGoalHandle, start_time):
        goal_handle.canceled()
        result = ExecuteMission.Result()
        result.success = False
        result.message = "cancelled by user"
        elapsed = self.get_clock().now() - start_time
        result.mission_duration.sec = int(elapsed.nanoseconds // 1_000_000_000)
        result.mission_duration.nanosec = int(
            elapsed.nanoseconds % 1_000_000_000
        )
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockMissionServer()
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
