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

struct BoundingBox {
    float upper_edge;
    float lower_edge;
    float left_edge;
    float right_edge;

    float score = 0.f;

    BoundingBox(tracking::Tracklet& bbox) {
        upper_edge = bbox.y + bbox.h / 2.f;
        lower_edge = boox.y - bbox.h / 2.f;
        left_edge = bbox.x - bbox.w / 2.f;
        right_edge = bbox.x + bbox.w / 2.f;
    }

    bool contains(BoundingBox& bebe) {
        return (papa.upper_edge > bebe.upper_edge && papa.lower_edge < bebe.lower_edge && 
            papa.left_edge < bebe.left_edge && papa.right_edge > bebe.right_edge)
    }
};


class SimpleTracker {


  public:
    SimpleTracker(ros::NodeHandle& n, int _enemy_color)
        : nh(n), enemy_color(_enemy_color) {
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

        /*float best_dist = INFINITY;
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
        }code de sébastien*/

        auto boxSize = [](auto bbox) {
            return bbox.w * bbox.h;
        }

        auto findboxes = [](auto bbox){
            std::vector <BoundingBox> enemy_boxes;
            BoundingBox papa(bbox);
            for (auto trk : trks->tracklets) {
                BoundingBox bebe(trk);
                if papa.contains(bebe){
                    enemy_boxes.push_back(bebe)
                }
            }
            return enemy_boxes;
        }

        
        
        auto roboType = [](bbox){
            found = False
            if bbox.clss == std {
                auto enemy_boxes = findBoxes(bbox);
                for (int i = 0; i < sizeof(enemy_boxes); i++) {
                    if enemy_boxes[i].clss == enemy {
                        found = True ;
                        }
                }
                if found {
                return 300;
                }
            }
            if bbox.clss == hero {
                auto enemy_boxes = findBoxes(bbox);
                for (int i = 0; i < sizeof(enemy_boxes); i++) {
                    if enemy_boxes[i].clss == enemy {
                        found = True ;
                        }
                }
                if found {
                return 1000;
                }
            }
                return 0;
        }
        
        float best_score = 0;
        int index = -1;

        int i = 0;
        for (auto trk : trks->tracklets) {
            auto dist = distance(last_trk, trk);
            auto size = boxSize(trk);
            auto type = roboType(trk);

            score += type;
            score += size / 8;
            score -= distance;
            if (score > best_score && enemy_color == int(trk.clss)) {
                index = i;
                best_score = score;
            }
            ++i;
        }
        if (index != -1) {
            last_trk = trks->tracklets[index];
            pub_target.publish(toTarget(last_trk));
        }
        //ca compilera pas mais c'est l'idée en général
        //On attribue on score selon le type du robot (hero > std), la distance du canon (proche > loin) et la taille de la bbox (car ++taille bbox = ++proche robot) 
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
