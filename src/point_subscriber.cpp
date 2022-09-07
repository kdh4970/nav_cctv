#include <ros/ros.h>
#include "nav_cctv/Locations.h"
#include "point_subscriber.h"

namespace PointSub{

  void pointCallback(const nav_cctv::LocationsConstPtr&Topic_point)
  {
    PointSub::Topic_X = Topic_point->location;
    PointSub::x_Ptr = &Topic_X;
    PointSub::msg_seq = Topic_point->msg_seq;
    PointSub::seq_Ptr = &msg_seq;
    ROS_INFO("point is %d, msg_seq is %d", PointSub::Topic_X,PointSub::msg_seq);
  }
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"point_subscriber");
    ros::NodeHandle point_nh;
    ros::Subscriber point_sub = point_nh.subscribe("points",5,PointSub::pointCallback);
    ros::spin();
    

    return 0;
}

