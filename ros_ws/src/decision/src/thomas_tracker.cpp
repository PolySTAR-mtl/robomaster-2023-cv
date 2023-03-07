/** \file thomas_tracker.cpp
 * \brief Simple targeting node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 * \author Thomas Petrie <thomas.petrie@polymtl.ca>
 */

// Std includes

#include <algorithm>

// ROS includes

#include <ros/ros.h>

#include "serial/Target.h"
#include "tracking/Tracklets.h"

struct BoundingBox {
    float upper_edge; // const
    float lower_edge;
    float left_edge;
    float right_edge;
    float x;
    float y;
    float score = 0.f;

    int clss;

    BoundingBox(tracking::Tracklet& bbox)
        : x(bbox.x), y(bbox.y) { // initialiser les valeurs
        y = bbox.y;
        x = bbox.x;
        upper_edge = bbox.y + bbox.h / 2.f;
        lower_edge = bbox.y - bbox.h / 2.f;
        left_edge = bbox.x - bbox.w / 2.f;
        right_edge = bbox.x + bbox.w / 2.f;
        clss = bbox.clss;
    }

    bool contains(BoundingBox& outer) {
        return (this->upper_edge > outer.y && this->lower_edge < outer.y &&
                this->left_edge < outer.x && this->right_edge > outer.x);
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

        auto boxSize = [](auto bbox) { return bbox.w * bbox.h; };

        auto findBoxes = [&](auto bbox) {
            std::vector<BoundingBox> enemy_boxes;
            BoundingBox papa(bbox);
            for (auto trk : trks->tracklets) {
                BoundingBox outer(trk);
                if (papa.contains(outer)) {
                    enemy_boxes.push_back(outer);
                }
            }
            return enemy_boxes;
        };

        auto roboType = [&](auto bbox) {
            bool found = false;
            if (bbox.clss == 4) // if car (standard)
            {
                auto enemy_boxes = findBoxes(bbox);
                for (int i = 0; i < enemy_boxes.size(); i++) {
                    if (enemy_boxes[i].clss == enemy_color) {
                        found = true;
                    }
                }
                if (found) {
                    return 400;
                }
            }
            if (bbox.clss == 5) // création d'une septième classe (hero) dans
                                // detection/data/dji.names si possible
            {
                auto enemy_boxes = findBoxes(bbox);
                for (int i = 0; i < sizeof(enemy_boxes); i++) {
                    if (enemy_boxes[i].clss == enemy_color) {
                        found = true;
                    }
                }
                if (found) {
                    return 1000;
                }
            }
            if (bbox.clss == 3) // if base
            {
                auto enemy_boxes = findBoxes(bbox);
                for (int i = 0; i < sizeof(enemy_boxes); i++) {
                    if (enemy_boxes[i].clss == enemy_color) {
                        found = true;
                    }
                }
                if (found) {
                    return 200;
                }
            }
            if (bbox.clss == 6) // juste robot (donc pour la sentry qu'on ne
                                // peut pas classer en ce moment)
            {
                auto enemy_boxes = findBoxes(bbox);
                for (int i = 0; i < sizeof(enemy_boxes); i++) {
                    if (enemy_boxes[i].clss == enemy_color) {
                        found = true;
                    }
                }
                if (found) {
                    return 300;
                }
            }
            return 0;
        };

        float best_score = 0;
        int index = -1;

        int i = 0;
        for (auto trk : trks->tracklets) {
            auto dist = distance(last_trk, trk);
            auto size = boxSize(trk);
            auto type = roboType(trk);

            trk.score += type;
            trk.score += size / 8;
            trk.score = trk.score - dist;
            if (trk.score > best_score && enemy_color == int(trk.clss)) {
                index = i;
                best_score = trk.score;
            }
            ++i;
        }
        if (index != -1) {
            last_trk = trks->tracklets[index];
            pub_target.publish(toTarget(last_trk));
        }
        // ca compilera pas mais c'est l'idée en général
        // On attribue on score selon le type du robot (hero > std), la distance
        // du canon (proche > loin) et la taille de la bbox (car ++taille bbox =
        // ++proche robot)
    };

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
