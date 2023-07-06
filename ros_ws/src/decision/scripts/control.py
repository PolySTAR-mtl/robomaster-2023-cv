"""
@file control.py
@brief Robot movement control strategy

@author Sébastien Darche <sebastien.darche@polymtl>
"""

import rospy
import numpy as np

from serial.msg import Target, Movement


class Controller:
    def __init__(self, rate=10, lost_target_timeout=2):
        # ROS setup
        self.sub_target = rospy.Subscriber(
            '/decision/target', Target, self.target_callback, queue_size=1)

        self.pub_movement = rospy.Publisher(
            '/serial/movement', Movement, queue_size=1)

        # Parameters
        self.rate = rate
        self.timeout = lost_target_timeout
        self.freq_search = 1
        self.follow_speed = 1
        self.scale = 1/15
        self.speed_noise = 0.05

        self.last_target = None
        self.last_order = Movement(
            stamp=rospy.get_rostime(), v_x=0, v_y=0, omega=0)
        self.pos_dead_reckoning = np.zeros((3, 1))

        self.step = 0

    def integrate(self):
        dt = 1 / self.rate

        order = self.last_order

        vel = np.array([[order.v_x],
                        [order.v_y],
                        [order.omega]])

        # The cinematic model is obviously wrong but I dont have time for now
        self.pos_dead_reckoning += dt * vel

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.step += 0.1
            self.move()
            rate.sleep()

    def target_callback(self, target: Target):
        if target.located:
            self.last_target = target

    def has_target(self):
        if self.last_target is None:
            return False

        if not self.last_target.located:
            return False

        duration = rospy.get_rostime() - self.last_target.stamp

        return duration.to_sec() < self.timeout

    def search(self):
        msg = self.last_order
        msg.v_x = self.draw()
        msg.v_y = self.draw()
        msg.omega += self.scale * np.sin(self.freq_search * self.step)

        self.pub_movement.publish(msg)

    def draw(self):
        return np.random.normal(scale=self.speed_noise)

    def follow_target(self):
        direction = -1 if self.last_target.phi < 0 else 1
        fixed_rate = self.follow_speed * direction

        msg = Movement(
            stamp=rospy.get_rostime(),
            v_x=0,
            v_y=0,
            omega=fixed_rate
        )

        self.pub_movement.publish(msg)
        self.last_order = msg

    def move(self):
        self.integrate()
        if not self.has_target():
            self.search()
        else:
            self.follow_target()
