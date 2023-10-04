/** \file weighted_tracker.cpp
 * \brief Simple targeting node
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 * \author Thomas Petrie <thomas.petrie@polymtl.ca>
 */

// Std includes

#include <algorithm>

// ROS includes

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// OpenCV Includes

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "serial/Target.h"
#include "tracking/Tracklets.h"

int16_t radToMillirad(float rad) { return static_cast<int16_t>(rad * 1000); }

enum class RoboType : int { Base = 3, Standard = 4, Hero = 5, Sentry = 6 };

struct BoundingBox;

struct BoundingBox {
    const float upper_edge; // const
    const float lower_edge;
    const float left_edge;
    const float right_edge;
    const float width;
    const float height;
    const float x;
    const float y;
    float score = 0.f;

    BoundingBox* parent;
    std::vector<BoundingBox> enfants;
    int nbEnfants;

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
        switch (this->clss) { //Si le robot est X on va dans la fonction scoreToReturn
        case static_cast<int>(RoboType::Base):
            return scoreReturn(enemy_color, weightBase, trks); 
        case static_cast<int>(RoboType::Standard):
            return scoreReturn(enemy_color, weightStandard, trks);
        case static_cast<int>(RoboType::Hero):
            return scoreReturn(enemy_color, weightHero, trks);
        case static_cast<int>(RoboType::Sentry):
            return scoreReturn(enemy_color, weightSentry, trks);
        default:
            return 0; //Si la bbox n'est pas englobante, on ne retourne rien pour le score du type de robot
        }
    }

    float scoreReturn(int enemy_color, float scoreToReturn,
                      const tracking::TrackletsConstPtr& trks) {

        std::vector<BoundingBox> enemy_boxes = this->findBoxes(trks); 
        //on va chercher toutes les bbox à l'intérieur de celle-ci

        bool found = false;

        for (auto& box : enemy_boxes) {
            if (box.clss == enemy_color) {    //si le module d'armure est de la couleur ennemie
                box.parent = this;            //on désigne le parent de ce module pour être celui-ci 
                this->enfants.push_back(box); //on ajoute aux enfants de cette bbox le module d'armure
                this->nbEnfants++;            //on incrémente le nombre d'enfants de cette bbox 
                found = true;
            }
        }

        if(found)                   //Si on a trouvé au moins un module d'armure dans la bbox 
            return scoreToReturn;   //On retourne le score qui équivaut à cette classe 
        return 0;                   //Sinon 0
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

class WeightedTracker {

  public:
    WeightedTracker(ros::NodeHandle& n, int _enemy_color)
        : tListener(tBuffer), nh(n), enemy_color(_enemy_color) {
        sub_tracklets = nh.subscribe("tracklets", 1,
                                     &WeightedTracker::callbackTracklets, this);

        pub_target = nh.advertise<serial::Target>("target", 1);

        // Init weights
        BoundingBox::weightBase = nh.param("weights/base", 200.f);
        BoundingBox::weightStandard = nh.param("weights/std", 400.f);
        BoundingBox::weightHero = nh.param("weights/hro", 1000.f);
        BoundingBox::weightSentry = nh.param("weights/sty", 300.f);
        BoundingBox::weightSize = nh.param("weights/size", 0.125);
        BoundingBox::weightDist = nh.param("weights/dist", -1.f);

        // Init camera matrix and distortion coefficients
        bool cam_param = true;
        cam_param &= nh.getParam("/camera/camera_matrix/data", camera_matrix);
        cam_param &= nh.getParam("/camera/distortion_coefficients/data",
                                 distorsion_coeffs);
        cam_param &= nh.getParam("/camera/image_width", im_w);
        cam_param &= nh.getParam("/camera/image_height", im_h);

        if (!cam_param) {
            throw std::runtime_error("WeightedTracker::WeightedTracker() : "
                                     "Could not fetch camera parameters");
        }

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

            std::cout << "Received Tracklet. \n" << "id: " << trk.id << 
            "x: "<< trk.x << "y: "<< trk.y << "w: "<< trk.w << "h: "<< 
            trk.h << "class: "<< trk.clss << "score: "<< trk.score;

            auto type = tracklet.roboType(enemy_color, trks); //Ici on définit également quelles bbox sont des enfants et des parents
            tracklet.score += type; //Le score du type est 0 si la bbox est un module d'armure

            if(tracklet.clss == 3 or tracklet.clss == 4 or tracklet.clss == 5 or tracklet.clss == 6){
                BoundingBox* best_target;
                float best_score = 0;

                for(BoundingBox enfant : tracklet.enfants){ //Pour tous les enfants d'une bbox parent
                    float size = enfant.getSize();          //On regarde laquelle a le meilleur score basé sur sa grandeur
                    auto dist = distance(last_trk, enfant); //et sur sa distance avec le dernier trk
                    enfant.score += size * BoundingBox::weightSize;
                    enfant.score += dist * BoundingBox::weightDist;
                    if(enfant.score > best_score){
                        best_target = &enfant;
                        best_score = enfant.score;
                    }
                }

                tracklet.score += best_score; //On ajoute au score du tracklet parent le score du meilleur enfant
            }
            

            if (tracklet.score > best_score) {  //On vise donc sur le robot avec le meilleur score
                index = i;                      //Ce qui combine son score de type et le score de son meilleur enfant pour la distance et la taille
                best_score = tracklet.score;
            }

            ++i;
        }

        if (index != -1) {
            last_trk = trks->tracklets[index];
            std::cout << "Published Tracklet. \n" << "id: " << last_trk.id << 
            "x: "<< last_trk.x << "y: "<< last_trk.y << "w: "<< last_trk.w << "h: "<< 
            last_trk.h << "class: "<< last_trk.clss << "score: "<< last_trk.score;
            pub_target.publish(toTarget(last_trk));
        }
    };

    serial::Target toTarget(tracking::Tracklet& trk) {
        serial::Target target;

        std::cout << "Det : " << trk.x << " ( " << trk.w << " ) " << trk.y
                  << " ( " << trk.h << " )\n";

        cv::Mat pixel_image({trk.x, trk.y});
        cv::Mat pixel_undistort(2, 1, CV_32FC1);

        pixel_undistort.at<float>(0) = mat1.at<float>(trk.y, trk.x);
        pixel_undistort.at<float>(1) = mat2.at<float>(trk.y, trk.x);

        cv::Mat x(cv::Point3f{pixel_undistort.at<float>(0),
                              pixel_undistort.at<float>(1), 1.f});

        cv::Mat y;
        cv::solve(new_c, x, y);

        std::cout << "solve\n" << y << '\n';

        y.at<float>(0) /= y.at<float>(2);
        y.at<float>(1) /= y.at<float>(2);

        tf2::Quaternion qTarget, qTurret;
        qTarget.setRPY(0., std::atan(y.at<float>(1)),
                       std::atan(y.at<float>(0)));

        auto transformTurret =
            tBuffer.lookupTransform("base_link", "turret", ros::Time(0));
        tf2::convert(transformTurret.transform.rotation, qTurret);

        qTarget *= qTurret;

        tf2::Matrix3x3 m(qTarget);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        int16_t theta = radToMillirad(pitch);
        int16_t phi = radToMillirad(yaw);

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
        new_c = cv::getOptimalNewCameraMatrix(c, d, im_size, 1.f, cv::Size(), 0,
                                              true);
        cv::initUndistortRectifyMap(c, d, cv::Mat(), new_c, im_size, CV_32F,
                                    mat1, mat2);

        std::cout << "c\n" << c << '\n';
        std::cout << "new_c\n" << new_c << '\n';

        im_center = cv::Mat(2, 1, CV_32F);
        im_center.at<float>(0) = new_c.at<float>(0, 2) / new_c.at<float>(0, 0);
        im_center.at<float>(1) = new_c.at<float>(1, 2) / new_c.at<float>(1, 1);

        std::cout << "im_center" << im_center << '\n';
    }

  private:
    tf2_ros::Buffer tBuffer;
    tf2_ros::TransformListener tListener;

    std::vector<float> camera_matrix;
    std::vector<float> distorsion_coeffs;

    cv::Mat new_c, mat1, mat2, im_center;

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

    WeightedTracker tracker(nh, enemy_color);

    ros::spin();
}

float BoundingBox::weightBase;
float BoundingBox::weightStandard;
float BoundingBox::weightHero;
float BoundingBox::weightSentry;
float BoundingBox::weightSize;
float BoundingBox::weightDist;
