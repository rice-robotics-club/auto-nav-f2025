#!/usr/bin/env python3
"""
keyboard_frame_estimator.py

ROS2 (Jazzy) Python node that takes 4 ArUco marker poses and computes a keyboard
reference frame (Transform) and broadcasts it with tf2.

Features:
- Accepts ArUco poses from a topic (PoseArray + ID array) or from a mock JSON file.
- Validates presence of marker IDs [4,5,6,7] (configurable).
- Verifies rectangularity (opposite sides roughly equal).
- Fits a plane to the 4 marker centers using SVD to compute the surface normal.
- Constructs a right-handed coordinate frame:
    - Origin: geometric center of the 4 markers.
    - X-axis: direction from marker 4 -> 5 (top-left -> top-right).
    - Y-axis: direction from marker 7 -> 4 (bottom-left -> top-left).
    - Z-axis: plane normal (right-hand rule).
- Builds a 4x4 transform and broadcasts it as 'keyboard_frame' in the camera frame.
- Docstrings and usage examples included below.

Usage (examples):
1) Run with live topics (default):
    ros2 run <your_package> keyboard_frame_estimator

    Provide poses on:
      /aruco_poses     : geometry_msgs/PoseArray
      /aruco_ids       : std_msgs/Int32MultiArray  (same ordering as poses)

2) Run using a mock JSON file (for testing):
    ros2 run <your_package> keyboard_frame_estimator --ros-args -p use_mock:=true -p mock_file:="/path/to/mock.json"

Mock file format (JSON):
{
  "ids": [4,5,6,7],
  "poses": [
    {"position": {"x":0.1,"y":0.2,"z":1.0}, "orientation": {"x":0,"y":0,"z":0,"w":1}},
    ...
  ]
}
Order of poses must correspond to the ids array.

Author: Generated assistant (adapt for your package).
"""

from __future__ import annotations
import math
import json
from typing import Dict, List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration

# Tolerances
DIST_TOLERANCE = 0.03  # meters relative tolerance for comparing sides (adjustable)
PLANE_SVD_TOL = 1e-6


def quat_to_rot_matrix(q: Tuple[float, float, float, float]) -> np.ndarray:
    """
    Convert quaternion (x,y,z,w) to 3x3 rotation matrix.
    """
    x, y, z, w = q
    # normalize first
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        # identity
        return np.eye(3)
    s = 2.0 / n
    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s

    R = np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ]
    )
    return R


def pose_to_xyz(p: Pose) -> np.ndarray:
    """Extract (x,y,z) as numpy array from geometry_msgs/Pose."""
    return np.array([p.position.x, p.position.y, p.position.z], dtype=float)


def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-12:
        return v
    return v / n


def fit_plane_svd(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Fit a plane to points (N x 3) using SVD.
    Returns (point_on_plane (centroid), normal_vector (unit)).
    """
    assert points.shape[0] >= 3, "Need at least 3 points to fit plane"
    centroid = points.mean(axis=0)
    # shift
    A = points - centroid
    # SVD
    u, s, vh = np.linalg.svd(A, full_matrices=False)
    # normal is last column of vh (or last row of vh.T)
    normal = vh[-1, :]
    normal = normalize(normal)
    return centroid, normal


def rotmat_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """
    Convert a 3x3 rotation matrix to quaternion (x,y,z,w).
    Uses numerically stable approach.
    """
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0  # S=4*qw
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S
        qz = (m[1, 0] - m[0, 1]) / S
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        S = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0  # S=4*qx
        qw = (m[2, 1] - m[1, 2]) / S
        qx = 0.25 * S
        qy = (m[0, 1] + m[1, 0]) / S
        qz = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0  # S=4*qy
        qw = (m[0, 2] - m[2, 0]) / S
        qx = (m[0, 1] + m[1, 0]) / S
        qy = 0.25 * S
        qz = (m[1, 2] + m[2, 1]) / S
    else:
        S = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0  # S=4*qz
        qw = (m[1, 0] - m[0, 1]) / S
        qx = (m[0, 2] + m[2, 0]) / S
        qy = (m[1, 2] + m[2, 1]) / S
        qz = 0.25 * S
    return (qx, qy, qz, qw)


def validate_rectangle(corners_xyz: Dict[int, np.ndarray], ids: List[int], tol: float = DIST_TOLERANCE) -> bool:
    """
    Validate that 4 corners form an approximate rectangle by checking that
    opposite sides are approximately equal in length.

    corners_xyz: dict id->xyz
    ids: list of four ids in the expected layout order:
         [top-left (4), top-right (5), bottom-right (6), bottom-left (7)]
    """
    if len(ids) != 4:
        return False

    a = corners_xyz[ids[0]]  # TL: 4
    b = corners_xyz[ids[1]]  # TR: 5
    c = corners_xyz[ids[2]]  # BR: 6
    d = corners_xyz[ids[3]]  # BL: 7

    # sides: AB (top), BC (right), CD (bottom), DA (left)
    AB = np.linalg.norm(b - a)
    BC = np.linalg.norm(c - b)
    CD = np.linalg.norm(d - c)
    DA = np.linalg.norm(a - d)

    # compare opposite sides AB vs CD, BC vs DA
    def approx_equal(x, y, tol_rel=tol):
        if x < 1e-6 and y < 1e-6:
            return True
        return abs(x - y) <= tol_rel * max(1.0, (x + y) / 2.0)

    ok1 = approx_equal(AB, CD)
    ok2 = approx_equal(BC, DA)
    return ok1 and ok2


class KeyboardFrameEstimator(Node):
    """
    ROS2 node to estimate keyboard TF from four ArUco poses.

    Parameters (ROS params):
      - marker_ids (list[int]): expected marker ids (default: [4,5,6,7])
      - aruco_pose_topic (str): topic that publishes PoseArray (default: /aruco_poses)
      - aruco_id_topic (str): topic that publishes Int32MultiArray of ids (default: /aruco_ids)
      - camera_frame (str): parent frame of keyboard (default: camera_frame)
      - keyboard_frame (str): child frame to broadcast (default: keyboard_frame)
      - use_mock (bool): if true, read mock_file and publish based on that (default: false)
      - mock_file (str): path to mock json file
    """

    def __init__(self):
        super().__init__("keyboard_frame_estimator")

        # Params (with defaults)
        self.declare_parameter("marker_ids", [4, 5, 6, 7])
        self.declare_parameter("aruco_pose_topic", "/aruco_poses")
        self.declare_parameter("aruco_id_topic", "/aruco_ids")
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("keyboard_frame", "keyboard_frame")
        self.declare_parameter("use_mock", False)
        self.declare_parameter("mock_file", "")

        self.marker_ids: List[int] = self.get_parameter("marker_ids").get_parameter_value().integer_array_value or [4, 5, 6, 7]
        # ROS2 integer_array_value returns ints differently in some clients; ensure python list
        if isinstance(self.marker_ids, (tuple, np.ndarray)):
            self.marker_ids = list(self.marker_ids)

        # fallback if parameter system returned nothing
        if not self.marker_ids:
            self.marker_ids = [4, 5, 6, 7]

        self.aruco_pose_topic = self.get_parameter("aruco_pose_topic").get_parameter_value().string_value or "/aruco_poses"
        self.aruco_id_topic = self.get_parameter("aruco_id_topic").get_parameter_value().string_value or "/aruco_ids"
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value or "camera_frame"
        self.keyboard_frame = self.get_parameter("keyboard_frame").get_parameter_value().string_value or "keyboard_frame"
        self.use_mock = self.get_parameter("use_mock").get_parameter_value().bool_value
        self.mock_file = self.get_parameter("mock_file").get_parameter_value().string_value or ""

        # broadcaster
        self.br = TransformBroadcaster(self)

        # storage
        # mapping id -> Pose
        self.poses_by_id: Dict[int, Pose] = {}

        # Subscribers
        # We'll subscribe to a PoseArray (poses) and a Int32MultiArray (ids)
        self.pose_sub = self.create_subscription(
            PoseArray, self.aruco_pose_topic, self._pose_array_callback, 10
        )
        self.id_sub = self.create_subscription(
            Int32MultiArray, self.aruco_id_topic, self._id_array_callback, 10
        )

        # In case of mock mode, we won't use subscribers; we'll load mock data on timer.
        if self.use_mock:
            self.get_logger().info(f"Using mock data from: {self.mock_file}")
            self.create_timer(0.5, self._publish_from_mock)
        else:
            # timer to attempt publishing keyboard frame at 20Hz
            self.create_timer(0.05, self._try_publish_keyboard_frame)

        # storage for last received list (ordered)
        self._last_received_ids: List[int] = []
        self._last_received_poses: List[Pose] = []

        self.get_logger().info("KeyboardFrameEstimator initialized")

    def _pose_array_callback(self, msg: PoseArray):
        """
        Expect PoseArray with poses in same order as ids published on /aruco_ids
        """
        self._last_received_poses = msg.poses

    def _id_array_callback(self, msg: Int32MultiArray):
        self._last_received_ids = list(msg.data)

    def _publish_from_mock(self):
        """
        Load mock json and behave as if we received PoseArray+IDs then attempt frame.
        """
        if not self.mock_file:
            self.get_logger().error("mock_file param not set")
            return
        try:
            with open(self.mock_file, "r") as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to read mock file: {e}")
            return

        ids = data.get("ids", [])
        poses = data.get("poses", [])
        if len(ids) != len(poses):
            self.get_logger().error("mock file 'ids' and 'poses' length mismatch")
            return

        # convert to Pose instances
        self._last_received_ids = []
        self._last_received_poses = []
        for i, pid in enumerate(ids):
            pose_dict = poses[i]
            pose = Pose()
            p = pose_dict.get("position", {})
            o = pose_dict.get("orientation", {"x": 0, "y": 0, "z": 0, "w": 1})
            pose.position.x = float(p.get("x", 0.0))
            pose.position.y = float(p.get("y", 0.0))
            pose.position.z = float(p.get("z", 0.0))
            pose.orientation.x = float(o.get("x", 0.0))
            pose.orientation.y = float(o.get("y", 0.0))
            pose.orientation.z = float(o.get("z", 0.0))
            pose.orientation.w = float(o.get("w", 1.0))
            self._last_received_ids.append(int(pid))
            self._last_received_poses.append(pose)

        # try publish
        self._try_publish_keyboard_frame()

    def _assemble_pose_map(self) -> Dict[int, Pose]:
        """
        Assemble mapping from id -> Pose using last received ids/poses lists.
        The user /aruco node must publish ids and poses in same order.
        """
        mapping: Dict[int, Pose] = {}
        ids = self._last_received_ids
        poses = self._last_received_poses
        if len(ids) != len(poses):
            # can't assemble
            return mapping
        for i, pid in enumerate(ids):
            mapping[int(pid)] = poses[i]
        return mapping

    def _try_publish_keyboard_frame(self):
        """
        Main routine: validate markers, compute frame, and broadcast transform.
        """
        mapping = self._assemble_pose_map()
        # Prefer mapping from individual lookups if available
        if not mapping:
            # fallback to poses_by_id (deprecated path)
            mapping = self.poses_by_id

        # Ensure all required markers exist
        missing = [m for m in self.marker_ids if m not in mapping]
        if missing:
            # not enough markers, skip
            self.get_logger().debug(f"Missing markers: {missing}")
            return

        # extract positions
        corners_xyz: Dict[int, np.ndarray] = {}
        for mid in self.marker_ids:
            p = mapping[mid]
            corners_xyz[mid] = pose_to_xyz(p)

        # validate rectangularity
        if not validate_rectangle(corners_xyz, self.marker_ids):
            self.get_logger().warn("Marker configuration failed rectangular validation. Skipping frame publish.")
            return

        # compute origin (geometric center)
        pts = np.vstack([corners_xyz[mid] for mid in self.marker_ids])
        origin = pts.mean(axis=0)

        # fit plane to points to get normal (Z axis)
        centroid, normal = fit_plane_svd(pts)

        # As defined: X-axis: direction from marker 4 -> 5 (marker_ids[0]->marker_ids[1])
        id_tl = self.marker_ids[0]
        id_tr = self.marker_ids[1]
        id_br = self.marker_ids[2]
        id_bl = self.marker_ids[3]

        x_dir = normalize(corners_xyz[id_tr] - corners_xyz[id_tl])
        # y_dir initially from BL -> TL per instruction "Y-axis: direction from marker 7 to 4 (bottom left to top left)"
        y_dir = normalize(corners_xyz[id_tl] - corners_xyz[id_bl])

        # ensure right-handed: z should be x cross y (note: right-hand rule)
        z_from_cross = normalize(np.cross(x_dir, y_dir))
        # if z_from_cross points opposite to fitted normal, flip so it matches plane normal direction
        if np.dot(z_from_cross, normal) < 0:
            z_from_cross = -z_from_cross

        # orthonormalize y to ensure perfect orthonormal basis: y = z x x
        y_orth = normalize(np.cross(z_from_cross, x_dir))

        # re-orthonormalize x (in case)
        x_orth = normalize(np.cross(y_orth, z_from_cross))

        R = np.column_stack((x_orth, y_orth, z_from_cross))  # 3x3 rotation matrix columns are axes

        # convert to quaternion
        qx, qy, qz, qw = rotmat_to_quaternion(R)

        # Build TransformStamped
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.keyboard_frame

        t.transform.translation.x = float(origin[0])
        t.transform.translation.y = float(origin[1])
        t.transform.translation.z = float(origin[2])

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        # broadcast
        self.br.sendTransform(t)
        self.get_logger().info_once("Published keyboard_frame transform (first time).")

    # Optional: public helper to compute frame from an explicit mapping id->Pose (useful for tests)
    def compute_keyboard_transform(self, id_pose_map: Dict[int, Pose]) -> Optional[TransformStamped]:
        """
        Compute and return a TransformStamped for the keyboard frame given an explicit
        id->Pose mapping. Returns None if validation fails.
        """
        # Check presence
        for mid in self.marker_ids:
            if mid not in id_pose_map:
                self.get_logger().error(f"Missing marker {mid} in supplied map")
                return None

        corners_xyz = {mid: pose_to_xyz(id_pose_map[mid]) for mid in self.marker_ids}

        if not validate_rectangle(corners_xyz, self.marker_ids):
            self.get_logger().error("Validation failed: markers not rectangular")
            return None

        pts = np.vstack([corners_xyz[mid] for mid in self.marker_ids])
        origin = pts.mean(axis=0)
        centroid, normal = fit_plane_svd(pts)

        id_tl = self.marker_ids[0]
        id_tr = self.marker_ids[1]
        id_br = self.marker_ids[2]
        id_bl = self.marker_ids[3]

        x_dir = normalize(corners_xyz[id_tr] - corners_xyz[id_tl])
        y_dir = normalize(corners_xyz[id_tl] - corners_xyz[id_bl])
        z_from_cross = normalize(np.cross(x_dir, y_dir))
        if np.dot(z_from_cross, normal) < 0:
            z_from_cross = -z_from_cross
        y_orth = normalize(np.cross(z_from_cross, x_dir))
        x_orth = normalize(np.cross(y_orth, z_from_cross))
        R = np.column_stack((x_orth, y_orth, z_from_cross))
        qx, qy, qz, qw = rotmat_to_quaternion(R)

        t = TransformStamped()
        # Use a non-stamped transform; user can set timestamps/frames before broadcasting
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.keyboard_frame
        t.transform.translation.x = float(origin[0])
        t.transform.translation.y = float(origin[1])
        t.transform.translation.z = float(origin[2])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        return t


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardFrameEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
