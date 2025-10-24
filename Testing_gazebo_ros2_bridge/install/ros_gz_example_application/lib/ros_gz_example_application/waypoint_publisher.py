#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # Create publisher for gnss_waypoint topic
        self.publisher_ = self.create_publisher(Point, '/gnss_waypoint', 10)
        
        # Create timer that triggers every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)
        
        self.get_logger().info('Waypoint Publisher Node has been started')
        self.get_logger().info('Publishing random waypoints every 5 seconds to /gnss_waypoint')
    
    def timer_callback(self):
        """Generate and publish random waypoint coordinates"""
        msg = Point()
        
        # Generate random x and y within [-10, 10] range
        msg.x = random.uniform(-10.0, 10.0)
        msg.y = random.uniform(-10.0, 10.0)
        msg.z = 0.0  # Ground level
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the published waypoint
        self.get_logger().info(f'Published waypoint: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    
    waypoint_publisher = WaypointPublisher()
    
    try:
        rclpy.spin(waypoint_publisher)
    except KeyboardInterrupt:
        pass
    
    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()