/** \file odom.cpp
 * \brief Odometry translator class implementation
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#include "odom.hpp"

void Odom::handlePos(float enc1, float enc2, float enc3, float enc4,
                     double dt) {
    Eigen::Vector4d enc(enc1, enc2, enc3, enc4);
    auto speed = enc - last_enc; // TODO : figure out direction!

    Eigen::Vector4d speed_smooth =
        smooth_factor * speed + (1 - smooth_factor) * last_speed;

    auto robot_vel = cinematic(speed_smooth);
    auto pos = integrate(robot_vel, dt);

    last_enc = enc;
    last_speed = speed;
}

Eigen::Vector3d Odom::cinematic(Eigen::Vector4d& vel) {
    auto v_x = (vel[0] + vel[1] + vel[2] + vel[3]) * wheel_radius / 4.;

    auto v_y =
        (-1. * vel[0] + vel[1] + vel[2] + -1. * vel[3]) * wheel_radius / 4.;

    auto omega = (-1. * vel[0] + vel[1] + -1 * vel[2] + vel[3]) * wheel_radius /
                 (4. * (length_x + length_y));

    return {v_x, v_y, omega};
}

Eigen::Vector3d Odom::integrate(Eigen::Vector3d& speed, double dt) {
    auto x = last_estimation.pose.pose.position.x + speed[0] * dt;
    auto y = last_estimation.pose.pose.position.y + speed[1] * dt;
    auto r = 0.; // TODO convert quaternion to euler

    return {x, y, r};
}

void Odom::handleSpeed(float v1, float v2, float v3, float v4) {}
