#include <ros/ros.h>
#include "nav_cctv/Point.h"


int main(int argc, char **argv)
{
    ros::init(argc,argv,"point_publisher");
    ros::NodeHandle nh;
    ros::Publisher point_pub = nh.advertise<nav_cctv::Point>("points",5);
    ros::Rate loop_rate(5);

    int count=0;
    while (ros::ok())
    {
        nav_cctv::Point pub_loc;
        pub_loc.msg_seq = count;
        pub_loc.x = 150;
        pub_loc.y = 100;
        ROS_INFO("msg_seq = %d",count);
        point_pub.publish(pub_loc);
        ++count;
        loop_rate.sleep();

    }
    

    return 0;
}