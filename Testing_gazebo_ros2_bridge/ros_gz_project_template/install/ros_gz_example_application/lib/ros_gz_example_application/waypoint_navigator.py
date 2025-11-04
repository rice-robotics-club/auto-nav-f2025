#!/usr/bin/env python3
"""
Waypoint Navigator Node for Differential Drive Rover

This node subscribes to waypoint commands and odometry data,
then publishes velocity commands to navigate the rover to the target waypoint
using simple proportional control.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class WaypointNavigator(Node):
    """
    A ROS 2 node that navigates a differential drive robot to waypoints
    using proportional control.
    """

    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Control parameters
        self.Kp_linear = 0.5  # Proportional gain for linear velocity
        self.Kp_angular = 2.0  # Proportional gain for angular velocity
        self.distance_tolerance = 0.3  # meters - how close is "reached"
        self.max_linear_vel = 0.5  # m/s - from SDF file
        self.max_angular_vel = 1.0  # rad/s - from SDF file
        
        # State variables
        self.current_position = None  # (x, y)
        self.current_heading = None  # yaw in radians
        self.target_waypoint = None  # (x, y)
        self.waypoint_active = False
        
        # Subscribers
        self.waypoint_sub = self.create_subscription(
            Point,
            '/gnss_waypoint',
            self.waypoint_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive/odometry',
            self.odometry_callback,
            10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/diff_drive/cmd_vel',
            10
        )
        
        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Waypoint Navigator Node initialized')
        self.get_logger().info(f'Control gains - Kp_linear: {self.Kp_linear}, Kp_angular: {self.Kp_angular}')
        self.get_logger().info(f'Distance tolerance: {self.distance_tolerance} meters')

    def waypoint_callback(self, msg):
        """
        Callback for receiving new waypoint commands.
        
        Args:
            msg (Point): Target waypoint with x, y, z coordinates (z ignored for 2D navigation)
        """
        self.target_waypoint = (msg.x, msg.y)
        self.waypoint_active = True
        self.get_logger().info(f'New waypoint received: x={msg.x:.2f}, y={msg.y:.2f}')

    def odometry_callback(self, msg):
        """
        Callback for receiving odometry data.
        Updates current position and heading of the rover.
        
        Args:
            msg (Odometry): Odometry message containing position and orientation
        """
        # Extract position (x, y)
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Extract orientation (convert quaternion to yaw)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_heading = yaw

    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        
        Args:
            angle (float): Angle in radians
            
        Returns:
            float: Normalized angle in range [-pi, pi]
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        """
        Clamp a value between min and max.
        
        Args:
            value (float): Value to clamp
            min_value (float): Minimum allowed value
            max_value (float): Maximum allowed value
            
        Returns:
            float: Clamped value
        """
        return max(min_value, min(value, max_value))

    def control_loop(self):
        """
        Main control loop that runs at 10 Hz.
        Calculates velocity commands based on current position and target waypoint.
        """
        # Check if we have all necessary data
        if self.current_position is None or self.current_heading is None:
            return
        
        # Check if we have an active waypoint
        if not self.waypoint_active or self.target_waypoint is None:
            return
        
        # Calculate errors
        dx = self.target_waypoint[0] - self.current_position[0]
        dy = self.target_waypoint[1] - self.current_position[1]
        distance_error = math.sqrt(dx**2 + dy**2)
        
        # Check if waypoint is reached
        if distance_error < self.distance_tolerance:
            # Stop the rover
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            
            self.get_logger().info(f'Waypoint reached! Final distance: {distance_error:.3f}m')
            self.waypoint_active = False
            return
        
        # Calculate desired heading (angle to goal)
        desired_heading = math.atan2(dy, dx)
        
        # Calculate heading error
        heading_error = self.normalize_angle(desired_heading - self.current_heading)
        
        # Calculate velocities using proportional control
        # Linear velocity - proportional to distance
        linear_vel = self.Kp_linear * distance_error
        linear_vel = self.clamp(linear_vel, -self.max_linear_vel, self.max_linear_vel)
        
        # Angular velocity - proportional to heading error
        angular_vel = self.Kp_angular * heading_error
        angular_vel = self.clamp(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Create and publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Log status for debugging
        self.get_logger().info(
            f'Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}) | '
            f'Target: ({self.target_waypoint[0]:.2f}, {self.target_waypoint[1]:.2f}) | '
            f'Distance: {distance_error:.2f}m | '
            f'Heading error: {math.degrees(heading_error):.1f}° | '
            f'Vel: lin={linear_vel:.2f}, ang={angular_vel:.2f}'
        )


def main(args=None):
    """Main entry point for the waypoint navigator node."""
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the rover before shutting down
        cmd_vel = Twist()
        navigator.cmd_vel_pub.publish(cmd_vel)
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()