#!/usr/bin/env python3

"""
1. get 4 aruco markers, compute geometric center
2. load coords from yaml -> keyboard frame -> convert to camera frame
3. publish to /key_poses
"""


import rclpy
from rclpy.node import Node
from aruco_detection_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PoseStamped


class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')

        # subscribe to aruco_markers topic to get aruco poses
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10
        )

        # create keyboard center publisher
        self.center_publisher = self.create_publisher(
            PoseStamped,
            '/keyboard_center',
            10
        )

        # marker IDs for the keyboard corners (see github readme)
        self.target_ids = [4, 5, 6, 7]

        self.get_logger().info('Keyboard detection node started. Listening for markers 4, 5, 6, 7...')

    def aruco_callback(self, msg):
        """
        Callback for ArucoMarkers messages.
        Filters for markers with IDs 4, 5, 6, 7 and computes their geometric center.
        """
        # Find poses for target marker IDs
        target_poses = []

        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id in self.target_ids:
                if i < len(msg.poses):
                    target_poses.append(msg.poses[i])
                    self.get_logger().debug(f'Found marker {marker_id} at index {i}')
        
        if not target_poses:
            self.get_logger().debug('No target markers (4, 5, 6, 7) detected in this frame')
            return
        if len(target_poses) < 4:
            self.get_logger().info(f'Only {len(target_poses)} target markers detected; need all 4 to compute keyboard center')
            # TODO: use partial info to estimate center if we have at least 2 markers


        # compute geometric center yahhhh
        sum_x = sum(pose.position.x for pose in target_poses)
        sum_y = sum(pose.position.y for pose in target_poses)
        sum_z = sum(pose.position.z for pose in target_poses)
        
        center_x = sum_x / len(target_poses)
        center_y = sum_y / len(target_poses)
        center_z = sum_z / len(target_poses)

        # make PoseStamped message
        center_msg = PoseStamped()
        center_msg.header = msg.header # header contains frame; we want to keep it in camera frame
        # other nodes (e.g. arm node) must transform to other frames if needed
        
        center_msg.pose.position.x = center_x
        center_msg.pose.position.y = center_y
        center_msg.pose.position.z = center_z

        # quaternions and orientations type shit
        center_msg.pose.orientation.x = sum(pose.orientation.x for pose in target_poses) / 4
        center_msg.pose.orientation.y = sum(pose.orientation.y for pose in target_poses) / 4
        center_msg.pose.orientation.z = sum(pose.orientation.z for pose in target_poses) / 4
        center_msg.pose.orientation.w = sum(pose.orientation.w for pose in target_poses) / 4

        
        self.center_publisher.publish(center_msg)

        self.get_logger().info(
            f'Keyboard center computed from {len(target_poses)} markers: '
            f'({center_x:.3f}, {center_y:.3f}, {center_z:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
