/** \file odom.cpp
 * \brief Odometry translator class definition
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"

class Odom {
  public:
    Odom(ros::NodeHandle& n) : nh(n) {
        pub_pos = nh.advertise<nav_msgs::Odometry>("odom", 1);
        pub_speed = nh.advertise<nav_msgs::Odometry>("odom_speed", 1);
    }

    void handlePos(float enc1, float enc2, float enc3, float enc4);
    void handleSpeed(float v1, float v2, float v3, float v4);

  public:
    ros::NodeHandle nh;
    ros::Publisher pub_pos, pub_speed;
};
