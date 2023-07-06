#!/usr/bin/env python
"""
@file robot_control.py
@brief General purpose robot control (movement, shooting)

@author Sébastien Darche <sebastien.darche@polymtl>
"""

import rospy

from shooting import Shooter
from control import Controller


if __name__ == '__main__':
    rospy.init_node('control', anonymous=False)
    detector = Shooter()
    controller = Controller()

    while not rospy.is_shutdown():
        controller.spin()
