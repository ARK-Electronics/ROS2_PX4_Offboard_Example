#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

KEY_UP = '\x41'
KEY_DOWN = '\x42'
KEY_RIGHT = '\x43'
KEY_LEFT = '\x44'

def get_key():
    file_descriptor = sys.stdin.fileno()
    old_settings = termios.tcgetattr(file_descriptor)
    try:
        tty.setraw(file_descriptor)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(file_descriptor, termios.TCSADRAIN, old_settings)
    return key

class KeyTeleopNode(Node):
    def __init__(self):
        super().__init__('key_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.read_key)

    def read_key(self):
        key = get_key()
        twist = Twist()

        if key == KEY_UP:
            twist.linear.x = 1.0
        elif key == KEY_DOWN:
            twist.linear.x = -1.0
        elif key == KEY_LEFT:
            twist.angular.z = 1.0
        elif key == KEY_RIGHT:
            twist.angular.z = -1.0
        else:
            return

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    key_teleop_node = KeyTeleopNode()

    try:
        rclpy.spin(key_teleop_node)
    except KeyboardInterrupt:
        pass

    key_teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
