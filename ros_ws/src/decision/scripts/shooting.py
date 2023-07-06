"""
@file shooting.py
@brief Sends / disables shooting orders for C&S

@author Sébastien Darche <sebastien.darche@polymtl>
"""

import rospy

from serial.msg import Target, Shoot


class Shooter:
    def __init__(self):
        # ROS setup
        self.sub_target = rospy.Subscriber(
            '/decision/target', Target, self.target_callback, queue_size=1)

        self.pub_shoot = rospy.Publisher('/serial/shoot', Shoot, queue_size=1)

        # State
        self.is_shooting = False
        self.count = 0

        # Parameters
        # TODO: rosparam / dynamic_reconfigure
        self.treshold = 30.  # Distance treshold before stopping shooting
        self.successive_tresh = 5  # Start shooting after n successive valid distances

    def target_callback(self, target: Target):
        if target.distance_center > self.treshold and self.is_shooting:
            # Too far. Stop shooting
            self.pub_shoot.publish(Shoot(shoot=False))

            # Update stat
            self.is_shooting = False
            self.count = 0

            return

        if target.distance_center < self.treshold:
            self.count += 1

            if self.count >= self.successive_tresh and not self.is_shooting:
                # Target acquired. Start shooting now
                self.is_shooting = True
                self.pub_shoot.publish(Shoot(shoot=True))
