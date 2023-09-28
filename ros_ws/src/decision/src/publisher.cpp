#include "ros/ros.h"
#include "tracking/Tracklets.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracklets");

  ros::NodeHandle nh("~");

  ros::Publisher chatter_pub = nh.advertise<Tracklets>("tracklets", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    Tracklets container = createTracklet("Papa", 0, 0, 100, 100, 0, 0);
    Tracklets contained = createTracklet("Bebe", 25, 25, 10, 10, 0, 0);  

    chatter_pub.publish(trk);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  return 0;
}

Tracklets createTracklet(string id, float32 x, float32 y, float32 w, 
                    float32 h, uint8 clss, float32 score){
    Tracklets trk;
    trk.id = id;
    trk.x = x;
    trk.y = y;
    trk.w = w;
    trk.h = h;
    trk.clss = clss;
    trk.score = score;

    return trk;
}