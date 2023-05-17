#!/usr/bin/env python3
import sys

import geometry_msgs.msg
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    # 'i': (1, 0, 0, 0),
    # 'o': (1, 0, 0, -1),
    # 'j': (0, 0, 0, 1),
    # 'l': (0, 0, 0, -1),
    # 'u': (1, 0, 0, 1),
    # ',': (-1, 0, 0, 0),
    # '.': (-1, 0, 0, 1),
    # 'm': (-1, 0, 0, -1),
    # 'O': (1, -1, 0, 0),
    # 'I': (1, 0, 0, 0),
    # 'J': (0, 1, 0, 0),
    # 'L': (0, -1, 0, 0),
    # 'U': (1, 1, 0, 0),
    # '<': (-1, 0, 0, 0),
    # '>': (-1, -1, 0, 0),
    # 'M': (-1, 1, 0, 0),
    'w': (0, 0, -1, 0), #Z+
    's': (0, 0, 1, 0),#Z-
    'a': (0, 0, 0, -1), #Yaw+
    'd': (0, 0, 0, 1),#Yaw-
    chr(65) : (1, 0, 0, 0),  #Up Arrow
    chr(66) : (-1, 0, 0, 0), #Down Arrow
    chr(67) : (0, 1, 0, 0), #Right Arrow
    chr(68) : (0, -1, 0, 0),  #Left Arrow
}

speedBindings = {
    # 'q': (1.1, 1.1),
    # 'z': (.9, .9),
    # 'w': (1.1, 1),
    # 'x': (.9, 1),
    # 'e': (1, 1.1),
    # 'c': (1, .9),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )


    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)

    speed = 0.5
    turn = .2
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            twist = geometry_msgs.msg.Twist()
            
            x_val = (x * speed) + x_val
            y_val = (y * speed) + y_val
            z_val = (z * speed) + z_val
            yaw_val = (th * turn) + yaw_val
            twist.linear.x = x_val
            twist.linear.y = y_val
            twist.linear.z = z_val
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = yaw_val
            pub.publish(twist)
            print(twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()