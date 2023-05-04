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

// OpenCV Includes

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "serial/Target.h"
#include "tracking/Tracklets.h"

int16_t radToMillirad(float rad) { return static_cast<int16_t>(rad * 1000); }

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
        return ((this->upper_edge > inner.y) && (this->lower_edge < inner.y) &&
                (this->left_edge < inner.x) && (this->right_edge > inner.x));
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
        nh.getParam("/camera/camera_matrix/data", camera_matrix);
        nh.getParam("/camera/distortion_coefficients/data", distorsion_coeffs);
        nh.getParam("/camera/image_width", im_w);
        nh.getParam("/camera/image_height", im_h);

        focal_length = nh.param("focal_length", 3.04e-3f);
        pixel_size = nh.param("pixel_size", 1.2e-6f);

        initMap();

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

        std::cout << "Det : " << trk.x << " ( " << trk.w << " ) " << trk.y
                  << " ( " << trk.h << " )\n";

        cv::Mat pixel_image({trk.x, trk.y});
        cv::Mat pixel_undistort(2, 1, CV_32FC1);

        pixel_undistort.at<float>(0) = mat1.at<float>(trk.x, trk.y);
        pixel_undistort.at<float>(1) = mat2.at<float>(trk.x, trk.y);

        cv::Mat x(cv::Point3f{pixel_undistort.at<float>(0),
                              pixel_undistort.at<float>(1), 1.f});

        cv::Mat y;
        cv::solve(new_c, x, y);

        // std::cout << "x_c = " << x_c << " ; y_c = " << y_c << '\n';

        int16_t theta = radToMillirad(std::atan(y.at<float>(0) / focal_length));
        int16_t phi = radToMillirad(std::atan(y.at<float>(1) / focal_length));

        std::cout << "    Trk : \n"
                  << pixel_image << "\n    Undistord\n"
                  << pixel_undistort << "\n    y\n"
                  << y << '\n';

        target.theta = theta;
        target.phi = -phi;
        target.dist = 2000u; // 2 m
        target.located = true;
        target.stamp = ros::Time::now();

        return target;
    }

    void initMap() {
        cv::Mat c(3, 3, CV_32F, camera_matrix.data());
        cv::Mat d(5, 1, CV_32F, distorsion_coeffs.data());
        cv::Point im_size{im_w, im_h};
        new_c = cv::getOptimalNewCameraMatrix(c, d, im_size, 0);
        cv::initUndistortRectifyMap(c, d, cv::Mat(), new_c, im_size, CV_32F,
                                    mat1, mat2);

        std::cout << new_c << '\n';
    }

  private:
    std::vector<float> camera_matrix;
    std::vector<float> distorsion_coeffs;

    cv::Mat new_c, mat1, mat2;

    ros::NodeHandle& nh;
    ros::Subscriber sub_tracklets;
    ros::Publisher pub_target;
    int enemy_color;

    tracking::Tracklet last_trk;

    int im_w;
    int im_h;

    float focal_length;
    float pixel_size;
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
