/** \file turret.hpp
 * \brief Turret position definition
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

#include "ros/ros.h"

#include "geometry_msgs/QuaternionStamped.h"
#include "serial/TurretFeedback.h"

class Turret {
  public:
    Turret(ros::NodeHandle& n) : nh(n) {
        pub_pos = nh.advertise<geometry_msgs::QuaternionStamped>("turret", 1);
    }

    void callbackTurret(const serial::TurretFeedbackPtr& turret);

  private:
    ros::NodeHandle nh;
    ros::Publisher pub_pos;
};
