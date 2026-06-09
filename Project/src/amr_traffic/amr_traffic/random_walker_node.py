#!/usr/bin/env python3
"""Random-walker controller for a single traffic robot.

Publishes /cmd_vel on the node's namespace based on a flat state machine
(FORWARD / TURN). Triggers transitions on:
  - random duration timeout
  - front-sector LiDAR obstacle (FORWARD -> TURN)
  - out-of-bounds soft check on odom (FORWARD -> TURN)

Run one instance per traffic robot, namespaced (e.g., ros2 run with __ns:=).
"""

import math
import random
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class RandomWalkerNode(Node):

    STATE_FORWARD = 'FORWARD'
    STATE_TURN = 'TURN'

    def __init__(self):
        super().__init__('random_walker_node')

        # ---- Parameters ----
        self.declare_parameter('forward_speed', 0.20)
        self.declare_parameter('turn_speed', 0.50)
        self.declare_parameter('obstacle_threshold', 1.20)
        self.declare_parameter('front_sector_half_angle_rad', 0.35)
        self.declare_parameter('forward_duration_min', 2.0)
        self.declare_parameter('forward_duration_max', 5.0)
        self.declare_parameter('turn_duration_min', 1.0)
        self.declare_parameter('turn_duration_max', 2.5)
        self.declare_parameter('bounds_radius', 25.0)
        self.declare_parameter('loop_rate_hz', 10.0)

        gp = self.get_parameter
        self.fwd_speed = float(gp('forward_speed').value)
        self.turn_speed = float(gp('turn_speed').value)
        self.obstacle_th = float(gp('obstacle_threshold').value)
        self.front_half = float(gp('front_sector_half_angle_rad').value)
        self.fwd_dur_min = float(gp('forward_duration_min').value)
        self.fwd_dur_max = float(gp('forward_duration_max').value)
        self.turn_dur_min = float(gp('turn_duration_min').value)
        self.turn_dur_max = float(gp('turn_duration_max').value)
        self.bounds_radius = float(gp('bounds_radius').value)
        self.loop_rate = float(gp('loop_rate_hz').value)

        # ---- State ----
        self.state = self.STATE_FORWARD
        self.state_until = self.get_clock().now() + Duration(
            seconds=random.uniform(self.fwd_dur_min, self.fwd_dur_max))
        self.turn_sign = 1
        self.front_blocked = False
        self.spawn_xy: Optional[Tuple[float, float]] = None
        self.current_xy: Optional[Tuple[float, float]] = None

        # ---- I/O ----
        # Topics are relative — the node is run inside the robot's namespace,
        # so /traffic_robot_1/cmd_vel etc. resolve via ROS_NAMESPACE.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self._scan_cb, SENSOR_QOS)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self._odom_cb, 10)

        period = 1.0 / max(self.loop_rate, 1.0)
        self.timer = self.create_timer(period, self._loop)

        self.get_logger().info(
            f'random_walker_node up: fwd={self.fwd_speed}m/s, '
            f'turn={self.turn_speed}rad/s, obstacle_th={self.obstacle_th}m, '
            f'bounds={self.bounds_radius}m'
        )

    # -------- Callbacks --------

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            self.front_blocked = False
            return

        amin = msg.angle_min
        ainc = msg.angle_increment
        rmin = msg.range_min
        rmax = msg.range_max

        # Wider net than naive ±half: also cover indices that wrap around 0
        # if the scan is centered differently. We compute angle for each index.
        nearest_front = math.inf
        for i in range(n):
            angle = amin + i * ainc
            # normalize to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            if abs(angle) <= self.front_half:
                r = msg.ranges[i]
                if math.isfinite(r) and rmin <= r <= rmax and r < nearest_front:
                    nearest_front = r

        self.front_blocked = (nearest_front < self.obstacle_th)

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_xy = (x, y)
        if self.spawn_xy is None:
            self.spawn_xy = (x, y)
            self.get_logger().info(
                f'spawn pose latched: ({x:.2f}, {y:.2f})'
            )

    # -------- Helpers --------

    def _out_of_bounds(self) -> bool:
        if self.spawn_xy is None or self.current_xy is None:
            return False
        sx, sy = self.spawn_xy
        cx, cy = self.current_xy
        return math.hypot(cx - sx, cy - sy) > self.bounds_radius

    def _enter_forward(self):
        self.state = self.STATE_FORWARD
        dur = random.uniform(self.fwd_dur_min, self.fwd_dur_max)
        self.state_until = self.get_clock().now() + Duration(seconds=dur)

    def _enter_turn(self):
        self.state = self.STATE_TURN
        self.turn_sign = random.choice([-1, 1])
        dur = random.uniform(self.turn_dur_min, self.turn_dur_max)
        self.state_until = self.get_clock().now() + Duration(seconds=dur)

    # -------- Main loop --------

    def _loop(self):
        now = self.get_clock().now()

        # Transition logic
        if self.state == self.STATE_FORWARD:
            timer_done = now >= self.state_until
            if timer_done or self.front_blocked or self._out_of_bounds():
                reason = (
                    'timer' if timer_done else
                    'obstacle' if self.front_blocked else
                    'bounds'
                )
                self.get_logger().debug(f'FORWARD -> TURN ({reason})')
                self._enter_turn()
        elif self.state == self.STATE_TURN:
            if now >= self.state_until:
                if self.front_blocked:
                    # Still pointing at obstacle — re-roll turn direction
                    # rather than driving into the wall.
                    self.get_logger().debug('TURN -> TURN (still blocked)')
                    self._enter_turn()
                else:
                    self.get_logger().debug('TURN -> FORWARD')
                    self._enter_forward()

        # Publish
        cmd = Twist()
        if self.state == self.STATE_FORWARD:
            cmd.linear.x = self.fwd_speed
        else:
            cmd.angular.z = self.turn_sign * self.turn_speed
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RandomWalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
