#!/usr/bin/env python

import rospy
import numpy as np

from serial.msg import PositionFeedback


def main():
    rospy.init_node('fake_serial')

    pub_odom = rospy.Publisher(
        '/serial/position', PositionFeedback, queue_size=1)
    rate = rospy.Rate(50)

    inc = 1

    t = 0.
    while not rospy.is_shutdown():
        msg = PositionFeedback(stamp=rospy.Time.now(),
                               imu_ax=0., imu_ay=0., imu_az=0.,
                               enc_1=t, enc_2=t, enc_3=t, enc_4=t, delta_t=rate.sleep_dur.to_sec())

        t += inc
        pub_odom.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
