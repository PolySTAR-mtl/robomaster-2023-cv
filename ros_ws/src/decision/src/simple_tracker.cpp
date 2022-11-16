/** \file simple_tracker.cpp
 * \brief Simple targeting node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 * \modifications Thomas Petrie <thomas.petrie@polymtl.ca> 
 */

// Std includes

#include <algorithm>

// ROS includes

#include <ros/ros.h>

#include "serial/Target.h"
#include "tracking/Tracklets.h"

class SimpleTracker {


  public:
    SimpleTracker(ros::NodeHandle& n, int _enemy_color /* int score*/)
        : nh(n), enemy_color(_enemy_color) /*score(score)*/ {
        sub_tracklets = nh.subscribe("tracklets", 1,
                                     &SimpleTracker::callbackTracklets, this);

        pub_target = nh.advertise<serial::Target>("target", 1);
        std::cout << "Enemy color set to be: "
                  << (enemy_color == 0 ? "red" : "blue") << "\n";
    }

    

    void callbackTracklets(const tracking::TrackletsConstPtr& trks) {
        auto distance = [](auto d1/*, auto d2*/) {
            return std::sqrt(std::pow(d1.x - /*d2.x*/0, 2) +
                             std::pow(d1.y - /*d2.y*/0, 2));
        };

        /*
        auto defineBox = [](bbox){
            upper_edge = bbox.y + height/2;
            lower_edge = boox.y - height/2;
            left_edge = bbox.x - width/2;
            right_edge = bbox.x + width/2;
        }
        
        auto boxSize = [](bbox) {
            return bbox.w * bbox.h;
        }
        
        auto roboType = [](bbox){
            if bbox.class == car:
                findBoxes(bbox);
                    
            On check pour tous les car/std/hero
            si ils ont des bbox enemie à l'int
            si oui on attirubue le score dépendamment du type de gros bbox
        }
        float best_score = INFINITY;
        int index = -1;

        int i = 0;
        for (auto trk : trks->tracklets) {
            auto dist = distance(trk);
            auto size = boxSize(trk);
            auto type = roboType(trk);

            score += type;
            score += size / 8;
            score -= distance;
            if (score < best_score && enemy_color == int(trk.clss)) {
                index = i;
                best_score = score;
            }
            ++i;
        }
        if (index != -1) {
            last_trk = trks->tracklets[index];
            pub_target.publish(toTarget(last_trk));
        }
        Mes méthodes à bien faire*/

        float best_dist = INFINITY;
        int index = -1;

        int i = 0;

        for (auto trk : trks->tracklets) {
            auto dist = distance(/*last trk, */ trk);
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

        auto x_c = trk.x + trk.w / 2 - im_w / 2;
        auto y_c = trk.y + trk.h / 2 - im_h / 2;

        std::cout << "x_c = " << x_c << " ; y_c = " << y_c << '\n';

        uint16_t theta = std::floor((y_c * alpha_y + M_PI_2) * 1000.f);
        int16_t phi = std::floor(x_c * alpha_x * 1000.f);

        target.theta = theta;
        target.phi = phi;
        target.dist = 2000u; // 2 m
        target.located = true;
        target.stamp = ros::Time::now();

        return target;
    }

  private:
    ros::NodeHandle& nh;
    ros::Subscriber sub_tracklets;
    ros::Publisher pub_target;
    int enemy_color;
    //int score;

    tracking::Tracklet last_trk;

    float im_w = 416 / 2;
    float im_h = 416 / 2;

    // Scaling factor
    float alpha_y = 0.001;
    float alpha_x = 0.01;
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

    ros::spin();
}
