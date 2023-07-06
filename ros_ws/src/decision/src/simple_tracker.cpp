/** \file simple_tracker.cpp
 * \brief Simple targeting node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

// Std includes

#include <algorithm>

// ROS includes

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "serial/Target.h"
#include "tracking/Tracklets.h"

#include "decision/DecisionConfig.h"
#include <dynamic_reconfigure/server.h>

class SimpleTracker {
  public:
    SimpleTracker(ros::NodeHandle& n, int _enemy_color)
        : nh(n), enemy_color(_enemy_color), tListener(tBuffer) {
        sub_tracklets = nh.subscribe("tracklets", 1,
                                     &SimpleTracker::callbackTracklets, this);

        pub_target = nh.advertise<serial::Target>("target", 1);
        std::cout << "Enemy color set to be: "
                  << (enemy_color == 0 ? "red" : "blue") << "\n";
    }

    void callbackTracklets(const tracking::TrackletsConstPtr& trks) {
        auto distance = [](auto d1, auto d2) {
            return std::sqrt(std::pow(d1.x - d2.x, 2) +
                             std::pow(d1.y - d2.y, 2));
        };
        float best_dist = INFINITY;
        int index = -1;

        int i = 0;

        for (auto trk : trks->tracklets) {
            auto dist = distance(last_trk, trk);
            if (dist < best_dist && enemy_color == int(trk.clss)) {
                index = i;
                best_dist = dist;
            }
            ++i;
        }

        if (index != -1) {
            last_trk = trks->tracklets[index];
            pub_target.publish(toTarget(last_trk));
        }
    }

    serial::Target toTarget(tracking::Tracklet& trk) {
        serial::Target target;

        std::cout << "Det : " << trk.x << " ( " << trk.w << " ) " << trk.y
                  << " ( " << trk.h << " )\n";

        auto x_c = static_cast<float>(trk.x + trk.w / 2.f - center_x);
        auto y_c = static_cast<float>(trk.y + trk.h / 2.f - center_y);

        std::cout << "x_c = " << x_c << " ; y_c = " << y_c << '\n';

        // Simple approximation .. if we consider x_c & y_c to be low enough
        int16_t theta = std::floor(y_c * alpha_y * 1000.f);
        int16_t phi = std::floor(x_c * alpha_x * 1000.f);

        tf2::Quaternion qTurret;

        try {
            auto transformTurret =
                tBuffer.lookupTransform("base_link", "turret", ros::Time(0));
            tf2::convert(transformTurret.transform.rotation, qTurret);
        } catch (tf2::LookupException e) {
            // Couldn't find lookup. Keep identity
            qTurret = tf2::Quaternion::getIdentity();
        }

        std::cout << qTurret << '\n';

        double roll, pitch, yaw;
        tf2::Matrix3x3 m(qTurret);
        m.getRPY(roll, pitch, yaw);

        std::cout << "Turret : p = " << pitch << ", y = " << yaw << '\n';

        target.theta = pitch - theta;
        target.phi = yaw + phi;
        target.dist = 2000u; // 2 m
        target.located = true;
        target.stamp = ros::Time::now();
        target.distance_center = std::hypot(x_c, y_c);

        return target;
    }

    void reconf(decision::DecisionConfig& config, uint32_t level) {
        enemy_color = config.enemy_color;
        center_x = config.trim_x;
        center_y = config.trim_y;

        std::cout << "New enemy color set to "
                  << (enemy_color == 0 ? "red" : "blue") << "\n";
        std::cout << "Trim : \n\tx : " << center_x << "\n\ty : " << center_y
                  << '\n';
    }

  private:
    ros::NodeHandle& nh;
    ros::Subscriber sub_tracklets;
    ros::Publisher pub_target;
    int enemy_color;

    tracking::Tracklet last_trk;

    float center_x = 416.f / 2.f;
    float center_y = 416.f / 2.f;

    // Scaling factor
    float alpha_y = 0.0007;
    float alpha_x = 0.0014;

    tf2_ros::Buffer tBuffer;
    tf2_ros::TransformListener tListener;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "decision");
    ros::NodeHandle nh("~");

    int enemy_color;

    if (!nh.getParam("enemy_color", enemy_color)) {
        throw std::runtime_error("Enemy color not specified");
    }
    if (enemy_color != 0 and enemy_color != 1) {
        throw std::runtime_error("Enemy color should be 0 (red) or 1 (blue)");
    }

    SimpleTracker tracker(nh, enemy_color);

    dynamic_reconfigure::Server<decision::DecisionConfig> server;
    server.setCallback(
        [&tracker](auto& c, auto level) { tracker.reconf(c, level); });

    ros::spin();
}
