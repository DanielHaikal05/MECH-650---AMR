#!/usr/bin/env python3
"""
Action client wrapper for the ExecuteMission action.

Phase 9 introduced this module. Master Plan §3.1 (amr_gui responsibilities).

Threading invariant: this client must be used only from the tkinter main thread.
ROS callbacks run on the same thread via rclpy.spin_once() called from a tkinter
timer (see mission_console.py). Do NOT use this client from a background thread.
"""

from __future__ import annotations

from typing import Callable, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.task import Future

from amr_mission_manager.action import ExecuteMission


class MissionActionClient:
    """Wraps the ROS 2 action client for ExecuteMission.

    Provides a tkinter-friendly callback interface: callers register Python
    functions that will be invoked (on the same thread as rclpy.spin_once)
    when goal acceptance, feedback, or result events arrive.
    """

    ACTION_NAME = "/execute_mission"

    def __init__(self, node: Node) -> None:
        self._node = node
        self._client = ActionClient(node, ExecuteMission, self.ACTION_NAME)
        self._goal_handle: Optional[ClientGoalHandle] = None

        # Callbacks registered by the GUI; default to no-ops.
        self.on_goal_accepted: Callable[[], None] = lambda: None
        self.on_goal_rejected: Callable[[str], None] = lambda reason: None
        self.on_feedback: Callable[[ExecuteMission.Feedback], None] = lambda fb: None
        self.on_result: Callable[[ExecuteMission.Result], None] = lambda res: None
        self.on_server_unavailable: Callable[[], None] = lambda: None

    def is_server_available(self, timeout_sec: float = 0.5) -> bool:
        """Returns True if the action server is reachable within the timeout."""
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def has_active_goal(self) -> bool:
        return self._goal_handle is not None

    def send_mission(self, mission_type: str, destination_house: str) -> bool:
        """Send a mission goal. Returns False if the server is unavailable.

        Returns True if the goal was sent (not necessarily accepted); the
        on_goal_accepted or on_goal_rejected callback will fire later.
        """
        if not self.is_server_available(timeout_sec=0.5):
            self._node.get_logger().warn(
                f"Action server '{self.ACTION_NAME}' is not available."
            )
            self.on_server_unavailable()
            return False

        goal_msg = ExecuteMission.Goal()
        goal_msg.mission_type = mission_type
        goal_msg.destination_house = destination_house

        send_goal_future: Future = self._client.send_goal_async(
            goal_msg, feedback_callback=self._handle_feedback
        )
        send_goal_future.add_done_callback(self._handle_goal_response)
        return True

    def cancel_mission(self) -> bool:
        """Cancel the active mission, if any. Returns False if no active goal."""
        if self._goal_handle is None:
            return False
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._handle_cancel_response)
        return True

    # ----- Internal callbacks -----

    def _handle_goal_response(self, future: Future) -> None:
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.on_goal_rejected("server rejected the goal")
            return
        self._goal_handle = goal_handle
        self.on_goal_accepted()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_result)

    def _handle_feedback(self, feedback_msg) -> None:
        # feedback_msg.feedback is the ExecuteMission.Feedback message
        self.on_feedback(feedback_msg.feedback)

    def _handle_result(self, future: Future) -> None:
        result_wrapper = future.result()
        # result_wrapper.result is the ExecuteMission.Result; .status is action GoalStatus
        self._goal_handle = None
        self.on_result(result_wrapper.result)

    def _handle_cancel_response(self, future: Future) -> None:
        # Cancel response is informational; the result callback will fire
        # separately with the cancelled outcome. No GUI work needed here.
        pass

    def destroy(self) -> None:
        """Release resources. Call before rclpy.shutdown()."""
        self._client.destroy()
