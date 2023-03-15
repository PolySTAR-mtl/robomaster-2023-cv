/** \file locate.cpp
 * \brief Locate node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#include "imu.hpp"
#include "odom.hpp"
#include "turret.hpp"

#include "ros/ros.h"

#include "serial/PositionFeedback.h"

void handleMessage(const serial::PositionFeedbackConstPtr& pos, IMU& imu,
                   Odom& odom) {
    imu.handle(pos->imu_ax, pos->imu_ay, pos->imu_az, pos->imu_rx, pos->imu_ry,
               pos->imu_rz);
    odom.handlePos(pos->enc_1, pos->enc_2, pos->enc_3, pos->enc_4,
                   pos->delta_t);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "locate");

    ros::NodeHandle nh;
    IMU imu{nh};
    Odom odom{nh};
    Turret turret{nh};

    auto sub_pos = nh.subscribe<serial::PositionFeedback>(
        "/serial/position", 1, [&imu, &odom](const auto& pos) -> void {
            handleMessage(pos, imu, odom);
        });

    auto sub_turret =
        nh.subscribe("/serial/turret", 1, &Turret::callbackTurret, &turret);

    ros::spin();

    return 0;
}
