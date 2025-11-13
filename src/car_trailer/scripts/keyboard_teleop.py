#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

# Define velocity step sizes
LIN_VEL_STEP_SIZE = 0.5
ANG_VEL_STEP_SIZE = 0.3

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        # Publisher for cmd_vel (for differential drive plugin)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.settings = termios.tcgetattr(sys.stdin)
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        w/s : increase/decrease linear velocity
        a/d : increase/decrease angular velocity (turn left/right)
        space : force stop
        
        CTRL-C or Esc to quit
        """

        print(msg)
        
        twist = Twist()

        try:
            while True:
                key = self.getKey()
                
                if key == '\x1b':  # Escape key
                    break
                elif key == '\x03':  # Ctrl-C
                    break
                elif key == ' ':  # Space - stop
                    self.linear_vel = 0.0
                    self.angular_vel = 0.0
                elif key == 'w':  # Forward
                    self.linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    self.linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'a':  # Turn Left
                    self.angular_vel += ANG_VEL_STEP_SIZE
                elif key == 'd':  # Turn Right
                    self.angular_vel -= ANG_VEL_STEP_SIZE

                # Limit velocities
                if self.linear_vel > 5.0:
                    self.linear_vel = 5.0
                if self.linear_vel < -5.0:
                    self.linear_vel = -5.0
                if self.angular_vel > 2.0:
                    self.angular_vel = 2.0
                if self.angular_vel < -2.0:
                    self.angular_vel = -2.0

                # Create and publish Twist message
                twist.linear.x = self.linear_vel
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.angular_vel

                self.cmd_vel_pub.publish(twist)
                
                # Print current velocities
                print(f"\rLinear: {self.linear_vel:.2f} m/s | Angular: {self.angular_vel:.2f} rad/s", end='', flush=True)

        except Exception as e:
            print(e)

        finally:
            # Stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        node.run_keyboard_control()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
