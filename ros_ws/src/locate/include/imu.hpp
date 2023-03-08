/** \file imu.cpp
 * \brief IMU translator class definition
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

#include "ros/ros.h"

#include "sensor_msgs/Imu.h"

class IMU {
  public:
    IMU(ros::NodeHandle& n) : nh(n) {
        pub_msg = nh.advertise<sensor_msgs::Imu>("imu", 1);
    }

    void handle(float ax, float ay, float az, float rx, float ry, float rz);

  private:
    ros::NodeHandle& nh;
    ros::Publisher pub_msg;
};
