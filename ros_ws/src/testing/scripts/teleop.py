#!/usr/bin/env python

"""
@file teleop.py
@brief Node to control the robot with the keyboard!

@details Inspired by the Turtlebot's teleop node

@author Sébastien Darche <sebastien.darche@polymtl.ca>
"""

import rospy

from serial.msg import Movement

import sys
import select
import termios
import tty
import numpy as np

settings = termios.tcgetattr(sys.stdin)

inc = 0.1  # Increment at each keystroke

# Stored as x, y, rot
mappings = {'w': np.array([inc, 0, 0]), 's': np.array(
    [-inc, 0, 0]), 'd': np.array([0, inc, 0]), 'a':  np.array([0, -inc, 0]),
    'e': np.array([0, 0, inc]), 'q':  np.array([0, 0, inc])}


def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

    key = None
    if rlist:
        key = sys.stdin.read(1)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    rospy.init_node("teleop")
    pub = rospy.Publisher('movement', Movement, queue_size=1)

    order = np.array([0., 0., 0.])

    try:
        while True:
            key = get_key()
            if key is None:
                pass
            elif key in mappings.keys():
                order += mappings[key]
            elif key == '\x03':
                break
            else:
                order = np.array([0., 0., 0.])

            pub.publish(Movement(v_x=order[0], v_y=order[1], omega=order[2]))

    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
