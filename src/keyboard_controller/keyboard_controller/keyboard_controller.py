#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('angular_speed', 0.5)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.get_logger().info("Use WASD keys to control the robot. Press 'q' to exit.")
        twist = Twist()
        while rclpy.ok():
            key = self.get_key()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if key:
                if 'w' in key:
                    twist.linear.x += self.linear_speed
                if 's' in key:
                    twist.linear.x -= self.linear_speed
                if 'a' in key:
                    twist.angular.z += self.angular_speed
                if 'd' in key:
                    twist.angular.z -= self.angular_speed
                if 'q' in key:
                    break

            self.publisher_.publish(twist)
            self.get_logger().info(f'Publishing: linear.x={twist.linear.x}, angular.z={twist.angular.z}')
        # Publish zero velocity before exiting
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
