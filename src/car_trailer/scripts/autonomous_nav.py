#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Controller gains
        self.k_linear = 0.5  # Linear velocity gain
        self.k_angular = 2.0  # Angular velocity gain
        
        # Goal tolerance
        self.goal_tolerance = 0.2  # meters
        
        # Waypoints to follow (x, y)
        self.waypoints = [
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0)
        ]
        
        self.current_waypoint_index = 0
        self.goal_reached = False
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Autonomous Controller Started')
        self.get_logger().info(f'Following {len(self.waypoints)} waypoints')

    def odom_callback(self, msg):
        """Update current pose from odometry"""
        # Get position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Get orientation (convert quaternion to yaw)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Main control loop - runs at 10 Hz"""
        if self.goal_reached:
            return
        
        # Check if we have reached all waypoints
        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info('All waypoints reached! Mission complete.')
            return
        
        # Get current goal
        goal_x, goal_y = self.waypoints[self.current_waypoint_index]
        
        # Calculate distance to goal
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if waypoint is reached
        if distance < self.goal_tolerance:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached!')
            self.current_waypoint_index += 1
            return
        
        # Calculate desired heading
        desired_theta = math.atan2(dy, dx)
        
        # Calculate heading error
        theta_error = self.normalize_angle(desired_theta - self.current_theta)
        
        # Create velocity command
        twist = Twist()
        
        # Linear velocity (proportional to distance, reduced when turning)
        if abs(theta_error) > 0.5:  # If large heading error, turn in place
            twist.linear.x = 0.0
        else:
            twist.linear.x = min(self.k_linear * distance, 1.0)  # Max 1.0 m/s
        
        # Angular velocity (proportional to heading error)
        twist.angular.z = self.k_angular * theta_error
        
        # Limit angular velocity
        max_angular = 1.5
        twist.angular.z = max(-max_angular, min(max_angular, twist.angular.z))
        
        # Publish command
        self.cmd_vel_pub.publish(twist)
        
        # Log progress
        if self.current_waypoint_index % 1 == 0:  # Log every waypoint
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                f'Distance: {distance:.2f}m, Heading error: {math.degrees(theta_error):.1f}°'
            )

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot stopped')

def main(args=None):
    rclpy.init(args=args)
    controller = AutonomousController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down autonomous controller')
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
