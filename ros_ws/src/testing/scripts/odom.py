#!/usr/bin/env python

import rospy
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
                               enc_1=t, enc_2=t, enc_3=t, enc_4=t, delta_t=rate.sleep_dur.to_sec())

        t += inc
        pub_odom.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
