/** \file odom.cpp
 * \brief Odometry translator class definition
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

#include "ros/ros.h"

#include "Eigen/Core"

#include "nav_msgs/Odometry.h"

class Odom {
  public:
    Odom(ros::NodeHandle& n) : nh(n) {
        pub_pos = nh.advertise<nav_msgs::Odometry>("odom", 1);
        pub_speed = nh.advertise<nav_msgs::Odometry>("odom_speed", 1);
    }

    /** \fn handlePos
     * \brief Handle a position message by derivating speed, applying the
     * cinematic model and integrating
     */
    void handlePos(float enc1, float enc2, float enc3, float enc4, double dt);

    /** \fn handleSpeed
     * \brief Apply the cinematic model
     */
    void handleSpeed(float v1, float v2, float v3, float v4);

    /** \fn cinematic
     * \brief 4-wheeled mecanum drive cinematic model
     */
    Eigen::Vector3d cinematic(Eigen::Vector4d& wheel_speed);

    /** \fn integrate
     * \brief Integrate the speed to obtain the new position
     */
    Eigen::Vector3d integrate(Eigen::Vector3d& robot_speed, double dt);

  private:
    ros::NodeHandle nh;
    ros::Publisher pub_pos, pub_speed;

    nav_msgs::Odometry last_estimation;

    double wheel_radius;
    double length_x, length_y;

    Eigen::Vector4d last_enc;
    Eigen::Vector4d last_speed;
    double smooth_factor = 0.95;
};
