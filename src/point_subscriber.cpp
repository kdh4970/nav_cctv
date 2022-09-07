#include <ros/ros.h>
#include "nav_cctv/Locations.h"
#include "point_subscriber.h"


int main(int argc, char **argv)
{
    ros::init(argc,argv,"point_subscriber");
    ros::NodeHandle point_nh;
    ros::Subscriber point_sub = point_nh.subscribe("points",5,pointCallback);
    ros::spin();
    

    return 0;
}

void pointCallback(const nav_cctv::LocationsConstPtr&Topic_point)
{
  *Topic_X = Topic_point->location;
  *msg_seq = Topic_point->msg_seq;
  ROS_INFO("point is %d, msg_seq is %d", *Topic_X,*msg_seq);
}