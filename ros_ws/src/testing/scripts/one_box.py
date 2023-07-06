#!/usr/bin/env python

import rospy
import numpy as np

from detection.msg import Detection, Detections


def main():
    rospy.init_node('fake_detections')

    pub_odom = rospy.Publisher(
        '/detection/detections', Detections, queue_size=1)

    freq = 10
    rate = rospy.Rate(freq)

    inc = 0.1
    t = 0.

    # describes a spiral

    amplitude_circle = 200.

    f_circle = 1
    f_amplitude = 30

    x_c = 416. / 2.
    y_c = 416. / 2.

    w = 30
    h = 20

    while not rospy.is_shutdown():
        rad = np.abs(amplitude_circle * np.cos(t * f_amplitude * inc / freq))

        x = rad * np.cos(t * f_circle * inc / freq)
        y = rad * np.sin(t * f_circle * inc / freq)
        det = Detection(x=x + x_c, y=y + y_c, w=w, h=h, score=1., clss=0)

        msg = Detections()
        msg.header.stamp = rospy.Time.now()
        msg.detections.append(det)

        t += inc
        pub_odom.publish(msg)

        print(rad)
        rate.sleep()


if __name__ == "__main__":
    main()
