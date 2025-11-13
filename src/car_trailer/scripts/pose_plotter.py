#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')
        
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Data storage
        self.timestamps = []
        self.x_positions = []
        self.y_positions = []
        self.theta_orientations = []
        
        self.start_time = None
        
        self.get_logger().info('Pose Plotter Started - Recording odometry data')
        self.get_logger().info('Press Ctrl+C to stop recording and generate plots')

    def odom_callback(self, msg):
        """Record pose data from odometry"""
        # Initialize start time
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds
        
        # Get position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Get orientation (convert quaternion to yaw)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        # Store data
        self.timestamps.append(elapsed)
        self.x_positions.append(x)
        self.y_positions.append(y)
        self.theta_orientations.append(theta)
        
        # Log every 1 second
        if len(self.timestamps) % 10 == 0:
            self.get_logger().info(
                f'Recording... Time: {elapsed:.1f}s, Pose: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)'
            )

    def generate_plots(self):
        """Generate and save plots"""
        if len(self.timestamps) == 0:
            self.get_logger().warn('No data to plot!')
            return
        
        self.get_logger().info(f'Generating plots with {len(self.timestamps)} data points...')
        
        # Create timestamp for filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Robot Pose Data Over Time', fontsize=16, fontweight='bold')
        
        # Plot 1: X position vs time
        axes[0, 0].plot(self.timestamps, self.x_positions, 'b-', linewidth=2)
        axes[0, 0].set_xlabel('Time (s)', fontsize=12)
        axes[0, 0].set_ylabel('X Position (m)', fontsize=12)
        axes[0, 0].set_title('X Position over Time', fontsize=14)
        axes[0, 0].grid(True, alpha=0.3)
        
        # Plot 2: Y position vs time
        axes[0, 1].plot(self.timestamps, self.y_positions, 'r-', linewidth=2)
        axes[0, 1].set_xlabel('Time (s)', fontsize=12)
        axes[0, 1].set_ylabel('Y Position (m)', fontsize=12)
        axes[0, 1].set_title('Y Position over Time', fontsize=14)
        axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: Theta (orientation) vs time
        theta_degrees = [math.degrees(t) for t in self.theta_orientations]
        axes[1, 0].plot(self.timestamps, theta_degrees, 'g-', linewidth=2)
        axes[1, 0].set_xlabel('Time (s)', fontsize=12)
        axes[1, 0].set_ylabel('Orientation (degrees)', fontsize=12)
        axes[1, 0].set_title('Orientation (Theta) over Time', fontsize=14)
        axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 4: X-Y trajectory (2D path)
        axes[1, 1].plot(self.x_positions, self.y_positions, 'purple', linewidth=2, label='Path')
        axes[1, 1].plot(self.x_positions[0], self.y_positions[0], 'go', markersize=10, label='Start')
        axes[1, 1].plot(self.x_positions[-1], self.y_positions[-1], 'ro', markersize=10, label='End')
        axes[1, 1].set_xlabel('X Position (m)', fontsize=12)
        axes[1, 1].set_ylabel('Y Position (m)', fontsize=12)
        axes[1, 1].set_title('2D Trajectory (X-Y Path)', fontsize=14)
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].legend(fontsize=10)
        axes[1, 1].axis('equal')
        
        plt.tight_layout()
        
        # Save figure
        filename = f'/home/shanthosh/Downloads/ros2_ws/pose_plot_{timestamp}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        self.get_logger().info(f'Plot saved to: {filename}')
        
        # Show plot
        plt.show()
        
        # Also save data to CSV
        csv_filename = f'/home/shanthosh/Downloads/ros2_ws/pose_data_{timestamp}.csv'
        with open(csv_filename, 'w') as f:
            f.write('Time(s),X(m),Y(m),Theta(rad),Theta(deg)\n')
            for i in range(len(self.timestamps)):
                f.write(f'{self.timestamps[i]:.3f},{self.x_positions[i]:.6f},'
                       f'{self.y_positions[i]:.6f},{self.theta_orientations[i]:.6f},'
                       f'{theta_degrees[i]:.3f}\n')
        self.get_logger().info(f'Data saved to CSV: {csv_filename}')
        
        # Print statistics
        self.get_logger().info('=== Pose Statistics ===')
        self.get_logger().info(f'Duration: {self.timestamps[-1]:.2f} seconds')
        self.get_logger().info(f'Total distance: {self.calculate_distance():.2f} meters')
        self.get_logger().info(f'X range: [{min(self.x_positions):.2f}, {max(self.x_positions):.2f}] m')
        self.get_logger().info(f'Y range: [{min(self.y_positions):.2f}, {max(self.y_positions):.2f}] m')

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
    plotter = PosePlotter()
    
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        plotter.get_logger().info('\nStopping data collection...')
        plotter.generate_plots()
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
