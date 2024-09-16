#!/usr/bin/env python3
import sys
import geometry_msgs.msg
import rclpy
import std_msgs.msg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform != 'win32':
    import termios
    import tty

welcome_message = """
This node takes keypresses from the keyboard and publishes them
as Twist messages.
Using the arrow keys and WASD you have Mode 2 RC controls.
W: Up
S: Down
A: Yaw Left
D: Yaw Right
Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

Press SPACE to arm/disarm the drone
"""

# Grouping moveBindings and function mappings into an array
controls = {
    'movement': {
        'w': (0, 0, 1, 0),   # Z+
        's': (0, 0, -1, 0),  # Z-
        'a': (0, 0, 0, -1),  # Yaw+
        'd': (0, 0, 0, 1),   # Yaw-
        '\x1b[A': (0, 1, 0, 0),  # Up Arrow
        '\x1b[B': (0, -1, 0, 0),  # Down Arrow
        '\x1b[C': (-1, 0, 0, 0),  # Right Arrow
        '\x1b[D': (1, 0, 0, 0),  # Left Arrow
    },
    'speed': {}
}

def getKey(settings):
    """Reads a key press and handles arrow key inputs."""
    if sys.platform == 'win32':
        return msvcrt.getwch()  # Windows: return single character
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)  # Read 1 char
        if key == '\x1b':  # Handle arrow keys
            key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def saveTerminalSettings():
    """Save the current terminal settings."""
    if sys.platform != 'win32':
        return termios.tcgetattr(sys.stdin)
    return None

def restoreTerminalSettings(old_settings):
    """Restore terminal settings."""
    if sys.platform != 'win32' and old_settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')

    # Set up QoS profile
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    # Publishers
    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)

    speed = 0.5
    turn = 0.2
    arm_toggle = False

    try:
        print(welcome_message)
        while True:
            key = getKey(settings)
            x, y, z, th = controls['movement'].get(key, (0, 0, 0, 0))

            if key == ' ':  # Arm/Disarm the drone with space key
                arm_toggle = not arm_toggle
                arm_msg = std_msgs.msg.Bool(data=arm_toggle)
                arm_pub.publish(arm_msg)
                print(f"Arm toggle is now: {arm_toggle}")

            if key == '\x03':  # CTRL-C to exit
                break

            # Create and publish twist message
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.z = th * turn
            pub.publish(twist)

            print(f"X: {twist.linear.x}  Y: {twist.linear.y}  Z: {twist.linear.z}  Yaw: {twist.angular.z}")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Send stop command on exit
        twist = geometry_msgs.msg.Twist()
        pub.publish(twist)
        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()
