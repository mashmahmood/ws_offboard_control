#!/usr/bin/env python3
import sys
import time
import select

import geometry_msgs.msg
import rclpy
import std_msgs.msg
from pynput import mouse

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
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

moveBindings = {
    'w': (0, 0, 1, 0), #Z+
    's': (0, 0, -1, 0),#Z-
    'a': (0, 0, 0, -1), #Yaw+
    'd': (0, 0, 0, 1),#Yaw-
    '\x1b[A' : (0, 1, 0, 0),  #Up Arrow
    '\x1b[B' : (0, -1, 0, 0), #Down Arrow
    '\x1b[C' : (-1, 0, 0, 0), #Right Arrow
    '\x1b[D' : (1, 0, 0, 0),  #Left Arrow
}


speedBindings = {
    # 'q': (1.1, 1.1),
    # 'z': (.9, .9),
    # 'w': (1.1, 1),
    # 'x': (.9, 1),
    # 'e': (1, 1.1),
    # 'c': (1, .9),
}

speed = 0.4
turn = 0.2


def getKey(settings, timeout=0.1):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':
                additional_chars = sys.stdin.read(2)
                key += additional_chars
        else:
            key = ''
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
    qos_profile_vel = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    )


    pub = node.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel_key', qos_profile_vel)

    arm_toggle = False
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)

    
    offboard_mode = True
    off_mode_pub = node.create_publisher(std_msgs.msg.Bool, '/offboard_mode', qos_profile)

    

    kbd_x = 0.0
    kbd_y = 0.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0
    

    mouse_mode = True
    mouse_dx = 0.0
    mouse_dy = 0.0
    last_mouse = [None, None]
    last_mouse_time = time.time()
    mouse_timeout = 0.3   # 300 ms (tune this)

    max_vel = 0.8
    mouse_sensitivity = 0.3   # tune this

    twist = geometry_msgs.msg.Twist()

    def on_move(xm, ym):
        nonlocal mouse_dx, mouse_dy, last_mouse, last_mouse_time

        if last_mouse[0] is None:
            last_mouse[0], last_mouse[1] = xm, ym
            return

        dx = xm - last_mouse[0]
        dy = ym - last_mouse[1]

        last_mouse[0], last_mouse[1] = xm, ym

        mouse_dx = dx
        mouse_dy = dy

        last_mouse_time = time.time()

    listener = mouse.Listener(on_move=on_move)
    listener.start()

    try:
        print(msg)
        # print(vels(speed, turn))

        while True:
            mouse_dx *= 0.8
            mouse_dy *= 0.8

            current_time = time.time()
            if current_time - last_mouse_time > mouse_timeout:
                mouse_dx = 0.0
                mouse_dy = 0.0

            if mouse_mode:
                mouse_x = -mouse_dx * mouse_sensitivity
                mouse_y =  -mouse_dy * mouse_sensitivity
            else:
                mouse_x = 0
                mouse_y = 0


            key = getKey(settings, timeout=0.1)
            if key == '\x03':  # Ctrl-C (ASCII)
                break

            if key == ' ':  # ASCII value for space
                arm_toggle = not arm_toggle  # Flip the value of arm_toggle
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_toggle
                arm_pub.publish(arm_msg)
                print(f"Arm toggle is now: {arm_toggle}")

            if key == 'o':
                offboard_mode = not offboard_mode  # Flip the value of offboard_mode
                off_mode_msg = std_msgs.msg.Bool()
                off_mode_msg.data = offboard_mode
                off_mode_pub.publish(off_mode_msg)
                print(f"Offboard Mode is now: {offboard_mode}")

            if key == 'm':
                mouse_mode = not mouse_mode  # Flip the value of mouse_mode
                print(f"Mouse Mode is now: {mouse_mode}")

            if key in moveBindings.keys():
                x, y, z, th = moveBindings[key]
                kbd_x = (x * speed)
                kbd_y = (y * speed)
                z_val = (z * speed) #+ z_val

                yaw_val = (th * turn) #+ yaw_val
                twist.angular.z = yaw_val
                
                
            
            # no keyboard
            elif key == '':
                kbd_x = 0.0
                kbd_y = 0.0
                z_val = 0.0
                twist.angular.z = 0.0
                
                # print("X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)
            

            x_val = kbd_x + mouse_x
            y_val = kbd_y + mouse_y
            x_val = max(min(x_val, max_vel), -max_vel)
            y_val = max(min(y_val, max_vel), -max_vel)

            twist.linear.x = x_val
            twist.linear.y = y_val
            twist.linear.z = z_val
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            print("X:",twist.linear.x, "   Y:",twist.linear.y, "   Z:",twist.linear.z, "   Yaw:",twist.angular.z)
            pub.publish(twist)
                

            

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
