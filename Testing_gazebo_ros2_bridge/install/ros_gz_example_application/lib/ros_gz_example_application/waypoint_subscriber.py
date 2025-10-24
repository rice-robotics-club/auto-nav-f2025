#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class WaypointSubscriber(Node):
    def __init__(self):
        super().__init__('waypoint_subscriber')
        
        # Create subscriber for gnss_waypoint topic
        self.subscription = self.create_subscription(
            Point,
            '/gnss_waypoint',
            self.waypoint_callback,
            10
        )
        
        self.get_logger().info('Waypoint Subscriber Node has been started')
        self.get_logger().info('Listening for waypoints on /gnss_waypoint')
    
    def waypoint_callback(self, msg):
        """Callback function that prints received waypoint coordinates"""
        self.get_logger().info(
            f'Received waypoint: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    
    waypoint_subscriber = WaypointSubscriber()
    
    try:
        rclpy.spin(waypoint_subscriber)
    except KeyboardInterrupt:
        pass
    
    waypoint_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()