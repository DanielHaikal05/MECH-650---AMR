#!/usr/bin/env python3

import math
import random
import csv
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan

try:
    from pyzbar.pyzbar import decode as pyzbar_decode
except Exception as exc:
    pyzbar_decode = None
    pyzbar_import_error = exc
else:
    pyzbar_import_error = None


class Explore(Node):
    def __init__(self):
        super().__init__("explore")

        # Parameters
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter(
            "image_topic",
            "/world/default/model/vehicle_blue/link/chassis/sensor/front_cam/image",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/world/default/model/vehicle_blue/link/chassis/sensor/front_cam/camera_info",
        )
        self.declare_parameter("qr_output_file", "qr_detections.csv")
        self.declare_parameter("qr_size", 2.0)
        self.declare_parameter("camera_horizontal_fov", 1.05)
        self.declare_parameter("camera_x", 0.2)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 0.95)
        self.declare_parameter("camera_roll", 0.0)
        self.declare_parameter("camera_pitch", -0.10)
        self.declare_parameter("camera_yaw", 0.0)
        self.declare_parameter("use_lidar_for_qr_position", True)
        self.declare_parameter("qr_lidar_window_deg", 4.0)
        self.declare_parameter("exploration_mode", "straight")
        self.declare_parameter("free_space_turn_gain", 0.35)
        self.declare_parameter("free_space_max_turn", 0.45)
        self.declare_parameter("random_turn_min_interval", 2.5)
        self.declare_parameter("random_turn_max_interval", 5.0)
        self.declare_parameter("random_turn_strength", 0.35)
        self.declare_parameter("small_rotation_min_interval", 2.0)
        self.declare_parameter("small_rotation_max_interval", 4.0)
        self.declare_parameter("small_rotation_duration", 0.45)
        self.declare_parameter("small_rotation_speed", 0.45)
        self.declare_parameter("landmark_scan_min_interval", 1.8)
        self.declare_parameter("landmark_scan_max_interval", 3.0)
        self.declare_parameter("landmark_scan_duration", 1.6)
        self.declare_parameter("landmark_scan_speed", 0.85)
        self.declare_parameter("landmark_drive_speed_scale", 0.9)
        self.declare_parameter("linear_speed", 0.70)
        self.declare_parameter("fast_linear_speed", 0.85)
        self.declare_parameter("turn_speed", 1.35)
        self.declare_parameter("straight_turn_speed", 0.85)
        self.declare_parameter("straight_angular_accel", 1.2)
        self.declare_parameter("safe_distance",1.5)
        self.declare_parameter("stop_distance", 0.8)
        self.declare_parameter("open_space_distance", 3.0)
        self.declare_parameter("front_angle_deg", 45.0)
        self.declare_parameter("side_angle_deg", 80.0)
        self.declare_parameter("camera_process_rate", 5.0)
        self.declare_parameter("control_period", 0.04)
        self.declare_parameter("escape_check_period", 4.0)
        self.declare_parameter("escape_min_progress", 0.45)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.qr_output_file = os.path.expanduser(
            self.get_parameter("qr_output_file").value
        )
        self.qr_size = float(self.get_parameter("qr_size").value)
        self.camera_horizontal_fov = float(
            self.get_parameter("camera_horizontal_fov").value
        )
        self.camera_translation_base = np.array([
            float(self.get_parameter("camera_x").value),
            float(self.get_parameter("camera_y").value),
            float(self.get_parameter("camera_z").value),
        ])
        self.camera_rotation_base = self.rotation_from_rpy(
            float(self.get_parameter("camera_roll").value),
            float(self.get_parameter("camera_pitch").value),
            float(self.get_parameter("camera_yaw").value),
        )
        self.use_lidar_for_qr_position = bool(
            self.get_parameter("use_lidar_for_qr_position").value
        )
        self.qr_lidar_window = math.radians(
            float(self.get_parameter("qr_lidar_window_deg").value)
        )
        self.exploration_mode = self.normalize_exploration_mode(
            self.get_parameter("exploration_mode").value
        )
        self.free_space_turn_gain = float(
            self.get_parameter("free_space_turn_gain").value
        )
        self.free_space_max_turn = max(
            float(self.get_parameter("free_space_max_turn").value),
            0.0,
        )
        self.random_turn_min_interval = max(
            float(self.get_parameter("random_turn_min_interval").value),
            0.1,
        )
        self.random_turn_max_interval = max(
            float(self.get_parameter("random_turn_max_interval").value),
            self.random_turn_min_interval,
        )
        self.random_turn_strength = max(
            float(self.get_parameter("random_turn_strength").value),
            0.0,
        )
        self.small_rotation_min_interval = max(
            float(self.get_parameter("small_rotation_min_interval").value),
            0.1,
        )
        self.small_rotation_max_interval = max(
            float(self.get_parameter("small_rotation_max_interval").value),
            self.small_rotation_min_interval,
        )
        self.small_rotation_duration = max(
            float(self.get_parameter("small_rotation_duration").value),
            0.05,
        )
        self.small_rotation_speed = max(
            float(self.get_parameter("small_rotation_speed").value),
            0.0,
        )
        self.landmark_scan_min_interval = max(
            float(self.get_parameter("landmark_scan_min_interval").value),
            0.1,
        )
        self.landmark_scan_max_interval = max(
            float(self.get_parameter("landmark_scan_max_interval").value),
            self.landmark_scan_min_interval,
        )
        self.landmark_scan_duration = max(
            float(self.get_parameter("landmark_scan_duration").value),
            0.1,
        )
        self.landmark_scan_speed = max(
            float(self.get_parameter("landmark_scan_speed").value),
            0.0,
        )
        self.landmark_drive_speed_scale = max(
            float(self.get_parameter("landmark_drive_speed_scale").value),
            0.0,
        )
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.fast_linear_speed = float(self.get_parameter("fast_linear_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.straight_turn_speed = float(
            self.get_parameter("straight_turn_speed").value
        )
        self.straight_angular_accel = max(
            float(self.get_parameter("straight_angular_accel").value),
            0.1,
        )
        self.safe_distance = float(self.get_parameter("safe_distance").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.open_space_distance = float(self.get_parameter("open_space_distance").value)
        self.front_angle = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.side_angle = math.radians(float(self.get_parameter("side_angle_deg").value))
        self.camera_process_period = 1.0 / max(
            float(self.get_parameter("camera_process_rate").value),
            0.1,
        )
        self.control_period = max(
            float(self.get_parameter("control_period").value),
            0.02,
        )
        self.escape_check_period = max(
            float(self.get_parameter("escape_check_period").value),
            1.0,
        )
        self.escape_min_progress = max(
            float(self.get_parameter("escape_min_progress").value),
            0.05,
        )

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            sensor_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            sensor_qos,
        )

        self.latest_scan = None
        self.last_scan_time = None
        self.latest_pose = None
        self.latest_pose_time = None
        self.latest_camera_info = None

        self.bridge = CvBridge()
        self.pyzbar_decode = pyzbar_decode
        self.use_opencv_qr = self.opencv_qr_has_quirc()
        self.qr_detector = cv2.QRCodeDetector() if self.use_opencv_qr else None
        self.qr_detections = {}
        self.qr_csv_header = [
            "qr_data",
            "qr_x",
            "qr_y",
            "qr_z",
        ]
        self.last_image_process_time = 0.0
        self.camera_frames_received = 0
        self.last_camera_log_time = 0.0
        self.logged_camera_info = False
        self.prepare_qr_output_file()

        self.escape_until = 0.0
        self.escape_direction = 1.0
        self.last_progress_pose = None
        self.last_progress_check_time = time.time()
        self.random_turn_bias = 0.0
        self.next_random_turn_time = self.next_random_turn_update_time()
        self.small_rotation_until = 0.0
        self.small_rotation_direction = 1.0
        self.next_small_rotation_time = self.next_small_rotation_update_time()
        self.landmark_scan_until = 0.0
        self.landmark_scan_direction = 1.0
        self.next_landmark_scan_time = self.next_landmark_scan_update_time()
        self.last_cmd_angular = 0.0
        self.last_cmd_time = time.time()

        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info(
            f"Wander SLAM node started in '{self.exploration_mode}' mode."
        )
        self.get_logger().info(f"Subscribing to: {self.scan_topic}")
        self.get_logger().info(f"Subscribing to: {self.odom_topic}")
        self.get_logger().info(f"Subscribing to: {self.image_topic}")
        self.get_logger().info(f"Subscribing to: {self.camera_info_topic}")
        self.get_logger().info(f"Publishing to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Saving QR detections to: {self.qr_output_file}")
        self.log_qr_decoder_status()

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg
        self.last_scan_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.latest_pose = msg.pose.pose
        self.latest_pose_time = self.get_clock().now()

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg
        if not self.logged_camera_info:
            self.logged_camera_info = True
            self.get_logger().info(
                "Camera intrinsics received from CameraInfo: "
                f"fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}, "
                f"cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f}"
            )

    def opencv_qr_has_quirc(self):
        try:
            build_info = cv2.getBuildInformation()
        except Exception:
            return False

        for line in build_info.splitlines():
            if "QUIRC" in line.upper():
                return "YES" in line.upper()

        return False

    def log_qr_decoder_status(self):
        if self.pyzbar_decode is not None:
            self.get_logger().info("QR decoder: pyzbar/ZBar enabled.")
        else:
            self.get_logger().warn(
                f"QR decoder: pyzbar/ZBar unavailable ({pyzbar_import_error}). "
                "Install python3-pyzbar and libzbar0."
            )

        if self.use_opencv_qr:
            self.get_logger().info("QR decoder: OpenCV QUIRC enabled.")
        else:
            self.get_logger().warn(
                "QR decoder: OpenCV QUIRC is not available; skipping "
                "cv2.QRCodeDetector to avoid 'Library QUIRC is not linked' spam."
            )

    def prepare_qr_output_file(self):
        output_dir = os.path.dirname(self.qr_output_file)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        with open(self.qr_output_file, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(self.qr_csv_header)

    def normalize_qr_payload(self, qr_data: str):
        return qr_data.strip().lower().replace(" ", "_")

    def image_callback(self, msg: Image):
        self.camera_frames_received += 1

        now = time.time()
        if now - self.last_camera_log_time > 5.0:
            self.last_camera_log_time = now
            self.get_logger().info(
                f"Camera frames received: {self.camera_frames_received}, "
                f"latest image: {msg.width}x{msg.height}, "
                f"QRs stored: {len(self.qr_detections)}"
            )

        if now - self.last_image_process_time < self.camera_process_period:
            return

        self.last_image_process_time = now

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Could not convert camera image: {exc}")
            return

        qr_detections = self.detect_qr_codes(frame)
        if not qr_detections:
            self.get_logger().debug(
                "Camera frame processed, but no QR code decoded.",
                throttle_duration_sec=2.0,
            )

        for detection in qr_detections:
            self.store_qr_detection(detection, msg)

    def detect_qr_codes(self, frame):
        detections = {}

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        equalized = cv2.equalizeHist(gray)
        threshold = cv2.adaptiveThreshold(
            equalized,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            41,
            5,
        )

        frames_to_try = [
            frame,
            gray,
            equalized,
            threshold,
        ]

        for candidate in frames_to_try:
            for qr_data, corners in self.decode_qr_frame(candidate):
                normalized_qr = self.normalize_qr_payload(qr_data)
                if normalized_qr not in detections or detections[normalized_qr][1] is None:
                    detections[normalized_qr] = (qr_data, corners)

        return list(detections.values())

    def decode_qr_frame(self, frame):
        detections = []

        if self.pyzbar_decode is not None:
            try:
                decoded_symbols = self.pyzbar_decode(frame)
                for symbol in decoded_symbols:
                    payload = symbol.data.decode("utf-8", errors="ignore").strip()
                    if payload:
                        corners = self.pyzbar_polygon_to_corners(symbol)
                        detections.append((payload, corners))
            except Exception as exc:
                self.get_logger().debug(
                    f"pyzbar failed to decode frame: {exc}",
                    throttle_duration_sec=2.0,
                )

        if self.qr_detector is None:
            return detections

        try:
            found, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(frame)
            if found:
                for index, data in enumerate(decoded_info):
                    if data and data.strip():
                        corners = None
                        if points is not None and index < len(points):
                            corners = self.order_image_points(points[index])
                        detections.append((data.strip(), corners))

            decoded_data, points, _ = self.qr_detector.detectAndDecode(frame)
            if decoded_data and decoded_data.strip():
                corners = self.order_image_points(points) if points is not None else None
                detections.append((decoded_data.strip(), corners))
        except cv2.error:
            pass

        return detections

    def pyzbar_polygon_to_corners(self, symbol):
        points = np.array(
            [[point.x, point.y] for point in symbol.polygon],
            dtype=np.float32,
        )

        if len(points) < 4:
            return None

        if len(points) > 4:
            rect = cv2.minAreaRect(points)
            points = cv2.boxPoints(rect)

        return self.order_image_points(points)

    def order_image_points(self, points):
        points = np.array(points, dtype=np.float32).reshape(-1, 2)
        if len(points) < 4:
            return None

        sums = points.sum(axis=1)
        diffs = np.diff(points, axis=1).reshape(-1)

        ordered = np.zeros((4, 2), dtype=np.float32)
        ordered[0] = points[np.argmin(sums)]
        ordered[2] = points[np.argmax(sums)]
        ordered[1] = points[np.argmin(diffs)]
        ordered[3] = points[np.argmax(diffs)]
        return ordered

    def camera_matrix(self, image_msg: Image):
        if self.latest_camera_info is not None:
            return np.array(self.latest_camera_info.k, dtype=np.float64).reshape(3, 3)

        self.get_logger().warn(
            "No CameraInfo received yet; using approximate intrinsics from FOV.",
            throttle_duration_sec=5.0,
        )

        width = float(image_msg.width)
        height = float(image_msg.height)
        fx = width / (2.0 * math.tan(self.camera_horizontal_fov / 2.0))
        fy = fx
        cx = width / 2.0
        cy = height / 2.0

        return np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    def distortion_coefficients(self):
        if self.latest_camera_info is None or not self.latest_camera_info.d:
            return np.zeros((4, 1), dtype=np.float64)

        return np.array(self.latest_camera_info.d, dtype=np.float64).reshape(-1, 1)

    def estimate_qr_position_from_sensors(self, corners, image_msg: Image):
        if corners is None:
            return None

        if self.latest_pose is None:
            return None

        half_size = self.qr_size / 2.0
        object_points = np.array([
            [-half_size, -half_size, 0.0],
            [half_size, -half_size, 0.0],
            [half_size, half_size, 0.0],
            [-half_size, half_size, 0.0],
        ], dtype=np.float64)

        image_points = np.array(corners, dtype=np.float64)
        camera_matrix = self.camera_matrix(image_msg)
        distortion = self.distortion_coefficients()

        success, _, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_IPPE,
        )

        if not success:
            return None

        qr_position_camera_optical = tvec.reshape(3)
        qr_position_camera_sdf = self.optical_to_sdf_camera(qr_position_camera_optical)
        pnp_position_base = (
            self.camera_translation_base
            + self.camera_rotation_base.dot(qr_position_camera_sdf)
        )
        qr_position_base = pnp_position_base
        pnp_distance = float(np.linalg.norm(qr_position_camera_optical))
        lidar_range = None
        estimate_source = "camera_pnp"

        lidar_position_base = self.estimate_qr_xy_from_lidar(image_points, image_msg)
        if lidar_position_base is not None:
            lidar_position_base[2] = pnp_position_base[2]
            qr_position_base = lidar_position_base
            lidar_range = float(np.linalg.norm(lidar_position_base[:2]))
            estimate_source = "camera_bearing_lidar_range"

        robot_position = np.array([
            self.latest_pose.position.x,
            self.latest_pose.position.y,
            self.latest_pose.position.z,
        ])
        robot_rotation = self.rotation_from_quaternion(self.latest_pose.orientation)
        qr_position_world = robot_position + robot_rotation.dot(qr_position_base)
        distance = float(np.linalg.norm(qr_position_base))

        return qr_position_world, distance, estimate_source, pnp_distance, lidar_range

    def estimate_qr_xy_from_lidar(self, image_points, image_msg: Image):
        if not self.use_lidar_for_qr_position or self.latest_scan is None:
            return None

        camera_matrix = self.camera_matrix(image_msg)
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        qr_center = image_points.mean(axis=0)
        ray_optical = np.array([
            (qr_center[0] - cx) / fx,
            (qr_center[1] - cy) / fy,
            1.0,
        ])
        ray_optical = ray_optical / np.linalg.norm(ray_optical)
        ray_camera_sdf = self.optical_to_sdf_camera(ray_optical)
        ray_base = self.camera_rotation_base.dot(ray_camera_sdf)

        horizontal_norm = math.hypot(ray_base[0], ray_base[1])
        if horizontal_norm < 1e-6:
            return None

        bearing = math.atan2(ray_base[1], ray_base[0])
        range_at_bearing = self.get_lidar_range_near_bearing(
            self.latest_scan,
            bearing,
            self.qr_lidar_window,
        )
        if range_at_bearing is None:
            return None

        # The lidar range is measured from the scan frame, which is effectively
        # the robot base for this Gazebo model. It gives a stronger x/y estimate
        # than monocular QR scale, while the camera still provides z.
        return np.array([
            range_at_bearing * math.cos(bearing),
            range_at_bearing * math.sin(bearing),
            0.0,
        ])

    def get_lidar_range_near_bearing(
        self,
        scan: LaserScan,
        bearing: float,
        window: float,
    ):
        candidates = []

        for i, scan_range in enumerate(scan.ranges):
            if math.isinf(scan_range) or math.isnan(scan_range):
                continue

            if scan_range < scan.range_min or scan_range > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            angle_error = abs(self.angle_difference(angle, bearing))
            if angle_error <= window:
                candidates.append((angle_error, scan_range))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[0])
        nearest_ranges = [scan_range for _, scan_range in candidates[:5]]
        nearest_ranges.sort()
        return nearest_ranges[len(nearest_ranges) // 2]

    def angle_difference(self, angle_a, angle_b):
        return math.atan2(
            math.sin(angle_a - angle_b),
            math.cos(angle_a - angle_b),
        )

    def optical_to_sdf_camera(self, point):
        # OpenCV optical frame: +Z forward, +X right, +Y down.
        # Gazebo camera frame used here: +X forward, +Y left, +Z up.
        return np.array([
            point[2],
            -point[0],
            -point[1],
        ])

    def rotation_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def rotation_from_rpy(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])

    def store_qr_detection(self, detection, image_msg: Image):
        qr_data, corners = detection
        normalized_qr = self.normalize_qr_payload(qr_data)

        if normalized_qr in self.qr_detections:
            return

        estimate = self.estimate_qr_position_from_sensors(corners, image_msg)
        if estimate is None:
            self.get_logger().warn(
                f"Detected QR '{qr_data}', but could not estimate its position "
                "from camera corners and odometry."
            )
            return

        qr_position_world, distance, estimate_source, _, _ = estimate
        qr_x = float(qr_position_world[0])
        qr_y = float(qr_position_world[1])
        qr_z = float(qr_position_world[2])

        self.qr_detections[normalized_qr] = {
            "qr_data": qr_data,
            "qr_x": qr_x,
            "qr_y": qr_y,
            "qr_z": qr_z,
        }

        with open(self.qr_output_file, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                qr_data,
                f"{qr_x:.6f}",
                f"{qr_y:.6f}",
                f"{qr_z:.6f}",
            ])

        self.get_logger().info(
            f"Stored QR '{qr_data}' from sensor estimate at "
            f"x={qr_x:.2f}, y={qr_y:.2f}, z={qr_z:.2f}, "
            f"distance={distance:.2f} m, source={estimate_source}"
        )

    def get_sector_min(self, scan: LaserScan, angle_min: float, angle_max: float) -> float:
        """
        Returns the minimum valid range in a sector.
        Angles are in radians, relative to the robot front.
        """
        ranges = scan.ranges
        valid_ranges = []

        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            if r < scan.range_min or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment

            if angle_min <= angle <= angle_max:
                valid_ranges.append(r)

        if not valid_ranges:
            return scan.range_max

        return min(valid_ranges)

    def publish_cmd(self, linear_x: float, angular_z: float):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    def normalize_exploration_mode(self, mode):
        valid_modes = {
            "straight",
            "free_space",
            "random",
            "random_small_rotation",
            "landmark_search",
        }
        normalized_mode = str(mode).strip().lower()
        if normalized_mode in valid_modes:
            return normalized_mode

        self.get_logger().warn(
            f"Unknown exploration_mode '{mode}'. Using 'straight'. "
            "Available modes: straight, free_space, random, "
            "random_small_rotation, landmark_search."
        )
        return "straight"

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def active_turn_speed(self):
        if self.exploration_mode == "straight":
            return self.straight_turn_speed

        return self.turn_speed

    def smooth_straight_angular(self, target_angular):
        if self.exploration_mode != "straight":
            self.last_cmd_angular = target_angular
            self.last_cmd_time = time.time()
            return target_angular

        now = time.time()
        dt = max(now - self.last_cmd_time, self.control_period)
        max_delta = self.straight_angular_accel * dt
        delta = self.clamp(
            target_angular - self.last_cmd_angular,
            -max_delta,
            max_delta,
        )

        self.last_cmd_angular += delta
        self.last_cmd_time = now
        return self.last_cmd_angular

    def next_random_turn_update_time(self):
        return time.time() + random.uniform(
            self.random_turn_min_interval,
            self.random_turn_max_interval,
        )

    def next_small_rotation_update_time(self):
        return time.time() + random.uniform(
            self.small_rotation_min_interval,
            self.small_rotation_max_interval,
        )

    def next_landmark_scan_update_time(self):
        return time.time() + random.uniform(
            self.landmark_scan_min_interval,
            self.landmark_scan_max_interval,
        )

    def clear_path_linear_speed(self, front_min):
        if front_min > self.open_space_distance:
            return self.fast_linear_speed

        clear_ratio = (front_min - self.safe_distance) / (
            self.open_space_distance - self.safe_distance
        )
        clear_ratio = self.clamp(clear_ratio, 0.0, 1.0)
        return self.linear_speed + clear_ratio * (
            self.fast_linear_speed - self.linear_speed
        )

    def clear_path_command(
        self,
        front_min,
        left_min,
        right_min,
        front_left_min,
        front_right_min,
    ):
        cmd_linear = self.clear_path_linear_speed(front_min)

        if self.exploration_mode == "free_space":
            return self.free_space_command(
                cmd_linear,
                left_min,
                right_min,
                front_left_min,
                front_right_min,
            )

        if self.exploration_mode == "random":
            return self.random_motion_command(cmd_linear)

        if self.exploration_mode == "random_small_rotation":
            return self.random_small_rotation_command(cmd_linear)

        if self.exploration_mode == "landmark_search":
            return self.landmark_search_command(cmd_linear, front_min)

        return cmd_linear, 0.0

    def free_space_command(
        self,
        cmd_linear,
        left_min,
        right_min,
        front_left_min,
        front_right_min,
    ):
        side_balance = left_min - right_min
        front_balance = front_left_min - front_right_min
        cmd_angular = self.free_space_turn_gain * (
            0.65 * side_balance + 0.35 * front_balance
        )
        cmd_angular = self.clamp(
            cmd_angular,
            -self.free_space_max_turn,
            self.free_space_max_turn,
        )
        return cmd_linear, cmd_angular

    def random_motion_command(self, cmd_linear):
        now = time.time()
        if now >= self.next_random_turn_time:
            self.random_turn_bias = random.uniform(
                -self.random_turn_strength,
                self.random_turn_strength,
            )
            self.next_random_turn_time = self.next_random_turn_update_time()

        return cmd_linear, self.random_turn_bias

    def random_small_rotation_command(self, cmd_linear):
        now = time.time()

        if now >= self.small_rotation_until and now >= self.next_small_rotation_time:
            self.small_rotation_direction = random.choice([-1.0, 1.0])
            self.small_rotation_until = now + self.small_rotation_duration
            self.next_small_rotation_time = self.next_small_rotation_update_time()

        if now < self.small_rotation_until:
            return (
                min(cmd_linear, self.linear_speed * 0.65),
                self.small_rotation_direction * self.small_rotation_speed,
            )

        return cmd_linear, 0.0

    def landmark_search_command(self, cmd_linear, front_min):
        now = time.time()

        if now >= self.landmark_scan_until and now >= self.next_landmark_scan_time:
            self.landmark_scan_direction = random.choice([-1.0, 1.0])
            self.landmark_scan_until = now + self.landmark_scan_duration
            self.next_landmark_scan_time = self.next_landmark_scan_update_time()

        if now < self.landmark_scan_until:
            return 0.0, self.landmark_scan_direction * self.landmark_scan_speed

        drive_speed = min(
            cmd_linear * self.landmark_drive_speed_scale,
            self.fast_linear_speed,
        )

        if front_min > self.open_space_distance:
            return drive_speed, 0.0

        return drive_speed, self.random_motion_command(0.0)[1] * 0.5

    def pose_distance(self, pose_a, pose_b):
        dx = pose_a.position.x - pose_b.position.x
        dy = pose_a.position.y - pose_b.position.y
        return math.hypot(dx, dy)

    def update_escape_behavior(self):
        now = time.time()

        if self.latest_pose is None:
            return

        if self.last_progress_pose is None:
            self.last_progress_pose = self.latest_pose
            self.last_progress_check_time = now
            return

        if now - self.last_progress_check_time < self.escape_check_period:
            return

        progress = self.pose_distance(self.latest_pose, self.last_progress_pose)
        self.last_progress_pose = self.latest_pose
        self.last_progress_check_time = now

        if progress >= self.escape_min_progress:
            return

        self.escape_direction *= -1.0
        self.escape_until = now + 1.4
        self.get_logger().info(
            f"Low exploration progress ({progress:.2f} m). Trying a new direction."
        )

    def control_loop(self):
        if self.latest_scan is None:
            self.stop_robot()
            return

        # Stop if scan data is old
        now = self.get_clock().now()
        if self.last_scan_time is None:
            self.stop_robot()
            return

        age = (now - self.last_scan_time).nanoseconds * 1e-9
        if age > 1.0:
            self.get_logger().warn("No recent /scan data. Stopping robot.")
            self.stop_robot()
            return

        scan = self.latest_scan
        self.update_escape_behavior()
        turn_speed = self.active_turn_speed()

        if time.time() < self.escape_until:
            cmd_angular = self.smooth_straight_angular(
                self.escape_direction * turn_speed
            )
            self.publish_cmd(0.0, cmd_angular)
            return

        # Main sectors
        front_min = self.get_sector_min(scan, -self.front_angle, self.front_angle)

        left_min = self.get_sector_min(
            scan,
            math.radians(25.0),
            self.side_angle,
        )

        right_min = self.get_sector_min(
            scan,
            -self.side_angle,
            math.radians(-25.0),
        )

        front_left_min = self.get_sector_min(
            scan,
            math.radians(0.0),
            self.front_angle,
        )

        front_right_min = self.get_sector_min(
            scan,
            -self.front_angle,
            math.radians(0.0),
        )

        cmd_linear = 0.0
        cmd_angular = 0.0

        # Emergency: very close obstacle
        if front_min < self.stop_distance:
            cmd_linear = -0.12

            # Turn toward the more open side
            if front_left_min < front_right_min:
                cmd_angular = -turn_speed
            else:
                cmd_angular = turn_speed

        # Obstacle ahead: turn away
        elif front_min < self.safe_distance:
            cmd_linear = 0.05

            if front_left_min < front_right_min:
                cmd_angular = -turn_speed
            else:
                cmd_angular = turn_speed

        # Path is clear: let the selected exploration mode decide how curious
        # the robot should be while keeping the same obstacle safety layer.
        else:
            cmd_linear, cmd_angular = self.clear_path_command(
                front_min,
                left_min,
                right_min,
                front_left_min,
                front_right_min,
            )

        cmd_angular = self.smooth_straight_angular(cmd_angular)
        self.publish_cmd(cmd_linear, cmd_angular)

        self.get_logger().info(
            f"mode={self.exploration_mode}, front={front_min:.2f}, "
            f"left={left_min:.2f}, right={right_min:.2f}, "
            f"v={cmd_linear:.2f}, w={cmd_angular:.2f}",
            throttle_duration_sec=1.0,
        )

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Explore()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
