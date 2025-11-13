#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import threading

class AutonomousWithPlotter(Node):
    def __init__(self):
        super().__init__('autonomous_with_plotter')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Controller gains
        self.k_linear = 1.2  # Linear velocity gain (increased for faster movement)
        self.k_angular = 3.0  # Angular velocity gain (increased for faster turning)
        
        # Goal tolerance
        self.goal_tolerance = 0.4  # meters (slightly larger for easier reaching)
        
        # Waypoints to follow (x, y) - Square path
        self.waypoints = [
            (3.0, 0.0),
            (3.0, 3.0),
            (0.0, 3.0),
            (0.0, 0.0)
        ]
        
        self.current_waypoint_index = 0
        self.goal_reached = False
        
        # Data storage for plotting
        self.timestamps = []
        self.x_positions = []
        self.y_positions = []
        self.theta_orientations = []
        self.start_time = None
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('='*60)
        self.get_logger().info('Autonomous Controller with Pose Plotter Started')
        self.get_logger().info(f'Following {len(self.waypoints)} waypoints in square path')
        self.get_logger().info('Recording pose data for plotting...')
        self.get_logger().info('='*60)

    def odom_callback(self, msg):
        """Update current pose from odometry and record data"""
        # Initialize start time
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        # Get position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Get orientation (convert quaternion to yaw)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Store data for plotting
        self.timestamps.append(elapsed)
        self.x_positions.append(self.current_x)
        self.y_positions.append(self.current_y)
        self.theta_orientations.append(self.current_theta)

    def control_loop(self):
        """Main control loop - runs at 10 Hz"""
        if self.goal_reached:
            return
        
        # Check if we have reached all waypoints
        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info('='*60)
            self.get_logger().info('All waypoints reached! Mission complete.')
            self.get_logger().info('='*60)
            self.get_logger().info('Generating pose plots...')
            # Generate plots in separate thread to avoid blocking
            threading.Thread(target=self.generate_plots).start()
            return
        
        # Get current goal
        goal_x, goal_y = self.waypoints[self.current_waypoint_index]
        
        # Calculate distance to goal
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if waypoint is reached
        if distance < self.goal_tolerance:
            self.get_logger().info(f'✓ Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} reached! -> ({goal_x}, {goal_y})')
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
            twist.linear.x = min(self.k_linear * distance, 2.0)  # Max 2.0 m/s for faster movement
        
        # Angular velocity (proportional to heading error)
        twist.angular.z = self.k_angular * theta_error
        
        # Limit angular velocity
        max_angular = 2.5
        twist.angular.z = max(-max_angular, min(max_angular, twist.angular.z))
        
        # Publish command
        self.cmd_vel_pub.publish(twist)

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

    def generate_plots(self):
        """Generate and save pose plots"""
        if len(self.timestamps) == 0:
            self.get_logger().warn('No data to plot!')
            return
        
        self.get_logger().info(f'Generating plots with {len(self.timestamps)} data points...')
        
        # Create timestamp for filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create figure with subplots - Required: X vs time and Y vs time
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Autonomous Navigation - Pose Data', fontsize=18, fontweight='bold')
        
        # Plot 1: X position vs time (REQUIRED)
        axes[0, 0].plot(self.timestamps, self.x_positions, 'b-', linewidth=2.5, label='X Position')
        axes[0, 0].set_xlabel('Time (seconds)', fontsize=14, fontweight='bold')
        axes[0, 0].set_ylabel('X Position (meters)', fontsize=14, fontweight='bold')
        axes[0, 0].set_title('X Position vs Time', fontsize=16, fontweight='bold')
        axes[0, 0].grid(True, alpha=0.3, linestyle='--')
        axes[0, 0].legend(fontsize=12)
        
        # Plot 2: Y position vs time (REQUIRED)
        axes[0, 1].plot(self.timestamps, self.y_positions, 'r-', linewidth=2.5, label='Y Position')
        axes[0, 1].set_xlabel('Time (seconds)', fontsize=14, fontweight='bold')
        axes[0, 1].set_ylabel('Y Position (meters)', fontsize=14, fontweight='bold')
        axes[0, 1].set_title('Y Position vs Time', fontsize=16, fontweight='bold')
        axes[0, 1].grid(True, alpha=0.3, linestyle='--')
        axes[0, 1].legend(fontsize=12)
        
        # Plot 3: Both X and Y vs time on same plot
        axes[1, 0].plot(self.timestamps, self.x_positions, 'b-', linewidth=2, label='X Position')
        axes[1, 0].plot(self.timestamps, self.y_positions, 'r-', linewidth=2, label='Y Position')
        axes[1, 0].set_xlabel('Time (seconds)', fontsize=14, fontweight='bold')
        axes[1, 0].set_ylabel('Position (meters)', fontsize=14, fontweight='bold')
        axes[1, 0].set_title('X and Y Position vs Time', fontsize=16, fontweight='bold')
        axes[1, 0].grid(True, alpha=0.3, linestyle='--')
        axes[1, 0].legend(fontsize=12)
        
        # Plot 4: X-Y trajectory (2D path)
        axes[1, 1].plot(self.x_positions, self.y_positions, 'purple', linewidth=3, label='Robot Path')
        axes[1, 1].plot(self.x_positions[0], self.y_positions[0], 'go', markersize=15, label='Start', zorder=5)
        axes[1, 1].plot(self.x_positions[-1], self.y_positions[-1], 'ro', markersize=15, label='End', zorder=5)
        
        # Plot waypoints
        waypoint_x = [w[0] for w in self.waypoints]
        waypoint_y = [w[1] for w in self.waypoints]
        axes[1, 1].plot(waypoint_x, waypoint_y, 'k*', markersize=20, label='Waypoints', zorder=4)
        
        axes[1, 1].set_xlabel('X Position (meters)', fontsize=14, fontweight='bold')
        axes[1, 1].set_ylabel('Y Position (meters)', fontsize=14, fontweight='bold')
        axes[1, 1].set_title('2D Trajectory (X-Y Path)', fontsize=16, fontweight='bold')
        axes[1, 1].grid(True, alpha=0.3, linestyle='--')
        axes[1, 1].legend(fontsize=12)
        axes[1, 1].axis('equal')
        
        plt.tight_layout()
        
        # Save figure
        filename = f'/home/shanthosh/Downloads/ros2_ws/autonomous_pose_plot_{timestamp}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Plot saved to: {filename}')
        
        # Also save data to CSV
        csv_filename = f'/home/shanthosh/Downloads/ros2_ws/autonomous_pose_data_{timestamp}.csv'
        with open(csv_filename, 'w') as f:
            f.write('Time(s),X(m),Y(m),Theta(rad),Theta(deg)\n')
            for i in range(len(self.timestamps)):
                theta_deg = math.degrees(self.theta_orientations[i])
                f.write(f'{self.timestamps[i]:.3f},{self.x_positions[i]:.6f},'
                       f'{self.y_positions[i]:.6f},{self.theta_orientations[i]:.6f},'
                       f'{theta_deg:.3f}\n')
        self.get_logger().info(f' Data saved to CSV: {csv_filename}')
        
        # Print statistics
        total_distance = self.calculate_distance()
        self.get_logger().info('='*60)
        self.get_logger().info(' Mission Statistics:')
        self.get_logger().info(f'   Duration: {self.timestamps[-1]:.2f} seconds')
        self.get_logger().info(f'   Total distance: {total_distance:.2f} meters')
        self.get_logger().info(f'   X range: [{min(self.x_positions):.2f}, {max(self.x_positions):.2f}] m')
        self.get_logger().info(f'   Y range: [{min(self.y_positions):.2f}, {max(self.y_positions):.2f}] m')
        self.get_logger().info(f'   Data points: {len(self.timestamps)}')
        self.get_logger().info('='*60)
        
        # Show plot
        plt.show()

    def calculate_distance(self):
        """Calculate total distance traveled"""
        total_distance = 0.0
        for i in range(1, len(self.x_positions)):
            dx = self.x_positions[i] - self.x_positions[i-1]
            dy = self.y_positions[i] - self.y_positions[i-1]
            total_distance += math.sqrt(dx**2 + dy**2)
        return total_distance

def main(args=None):
    rclpy.init(args=args)
    controller = AutonomousWithPlotter()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print('\n  Shutting down autonomous controller')
        try:
            controller.stop_robot()
        except:
            pass
        if len(controller.timestamps) > 0:
            print('Generating plots from partial data...')
            controller.generate_plots()
    finally:
        try:
            controller.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
