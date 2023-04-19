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

enum class RoboType : int { Base = 3, Standard = 4, Hero = 5, Sentry = 6 };

struct BoundingBox {
    const float upper_edge; // const
    const float lower_edge;
    const float left_edge;
    const float right_edge;
    const float width;
    const float height;
    const float x;
    const float y;
    const float score = 0.f;

    int clss;

    // Static weights loaded from ROS
    static float weightBase;
    static float weightStandard;
    static float weightHero;
    static float weightSentry;
    static float weightSize;
    static float weightDist;

    int getSize() { return this->width * this->height; }

    float roboType(int enemy_color, const tracking::TrackletsConstPtr& trks) {
        switch (this->clss) {
        case static_cast<int>(RoboType::Base):
            return scoreReturn(enemy_color, weightBase, trks);
        case static_cast<int>(RoboType::Standard):
            return scoreReturn(enemy_color, weightStandard, trks);
        case static_cast<int>(RoboType::Hero):
            return scoreReturn(enemy_color, weightHero, trks);
        case static_cast<int>(RoboType::Sentry):
            return scoreReturn(enemy_color, weightSentry, trks);
        default:
            return 0;
        }
    }

    float scoreReturn(int enemy_color, float scoreToReturn,
                      const tracking::TrackletsConstPtr& trks) {
        bool found = false;
        std::vector<BoundingBox> enemy_boxes = this->findBoxes(trks);

        for (const auto& box : enemy_boxes) {
            if (box.clss == enemy_color) {
                return scoreToReturn;
            }
        }

        return 0;
    }

    std::vector<BoundingBox>
    findBoxes(const tracking::TrackletsConstPtr& trks) {
        std::vector<BoundingBox> enemy_boxes;
        for (auto trk : trks->tracklets) {
            BoundingBox outer(trk);
            if (this->contains(outer)) {
                enemy_boxes.push_back(outer);
            }
        }
        return enemy_boxes;
    }

    BoundingBox(tracking::Tracklet& bbox)
        : x(bbox.x), y(bbox.y), upper_edge(bbox.y + bbox.h / 2.f),
          lower_edge(bbox.y - bbox.h / 2.f), left_edge(bbox.x - bbox.w / 2.f),
          right_edge(bbox.x + bbox.w / 2.f), clss(bbox.clss), width(bbox.w),
          height(bbox.h) {}

    bool contains(BoundingBox& inner) {
        return (this->upper_edge > inner.y && this->lower_edge < inner.y &&
                this->left_edge < inner.x && this->right_edge > inner.x);
    }
};

class SimpleTracker {

  public:
    SimpleTracker(ros::NodeHandle& n, int _enemy_color)
        : nh(n), enemy_color(_enemy_color) {
        sub_tracklets = nh.subscribe("tracklets", 1,
                                     &SimpleTracker::callbackTracklets, this);

        pub_target = nh.advertise<serial::Target>("target", 1);

        // Init weights
        BoundingBox::weightBase = nh.param("weights/base", 200.f);
        BoundingBox::weightStandard = nh.param("weights/std", 400.f);
        BoundingBox::weightHero = nh.param("weights/hro", 1000.f);
        BoundingBox::weightSentry = nh.param("weights/sty", 300.f);
        BoundingBox::weightSize = nh.param("weights/size", 0.125);
        BoundingBox::weightDist = nh.param("weights/dist", -1.f);

        // Init camera matrix and distortion coefficients
        matriceCamera = {{nh.param("camera_matrix/focale_x", 1252.05575f), 0.0f,
                          nh.param("camera_matrix/centre_x", 501.770228f)},
                         {0.0f, nh.param("camera_matrix/focale_y", 1328.81294f),
                          nh.param("camera_matrix/centre_y", 371.746642f)},
                         {0.0f, 0.0f, 1.0f}};
        matriceDistortion = {
            nh.param("coefficients_distortion/k1", 0.0604408241f),
            nh.param("coefficients_distortion/k2", 2.12039163f),
            nh.param("coefficients_distortion/p1", 0.00371581404f),
            nh.param("coefficients_distortion/p2", -0.0330357264f),
            nh.param("coefficients_distortion/k3", -12.3306280f)};

        std::cout << "Enemy color set to be: "
                  << (enemy_color == 0 ? "red" : "blue") << "\n";
    }

    void callbackTracklets(const tracking::TrackletsConstPtr& trks) {
        auto distance = [](auto d1, auto d2) {
            return std::sqrt(std::pow(d1.x - d2.x, 2) +
                             std::pow(d1.y - d2.y, 2));
        };

        float best_score = 0;
        int index = -1;

        int i = 0;
        for (auto trk : trks->tracklets) {
            BoundingBox tracklet(trk);

            auto dist = distance(last_trk, trk);
            float size = tracklet.getSize();
            auto type = tracklet.roboType(enemy_color, trks);

            auto score = type;
            score += size * BoundingBox::weightSize;
            score += dist * BoundingBox::weightDist;

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
    };

    serial::Target toTarget(tracking::Tracklet& trk) {
        serial::Target target;

        // La matrice calculée sur la caméra Jetson est
        //(1/focale x , 0 , centre x)     (1.25205575e+03 , 0 , 5.01770228e+02)
        //(0 , 1/focale y , centre y)  =  (0 , 1.32881294e+03 , 3.71746642e+02)
        //(0 , 0 , 1)                   (0, 0, 1)

        // La distance focale est en m. Un pixel est 1.12 um
        // float distance_focale_x = (1 / 1252.05575) / (0.00000112);
        // float distance_focale_y = (1 / 1328.81294) / (0.00000112);

        // La taille (en m) de l'image est de 3280 pixels * 1.12 micromètres par
        // pixel en x Et de 2464 pixels * 1.12 micromètres par pixel
        // float centre_image_x = 0.0018368;
        // float centre_image_y = 0.00137984;

        // float centre_bbox_x = trk.x + trk.w / 2;
        // float centre_bbox_y = trk.y + trk.h / 2;

        // float theta = std::atan(centre_bbox_x - centreX / focaleX);
        // float phi = std::atan(centre_bbox_y - centreY / focaleY);

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

    // Static weights loaded from ROS
  private:
    static std::vector<std::vector<float>> matriceCamera;
    static std::vector<float> matriceDistortion;

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

float BoundingBox::weightBase;
float BoundingBox::weightStandard;
float BoundingBox::weightHero;
float BoundingBox::weightSentry;
float BoundingBox::weightSize;
float BoundingBox::weightDist;
