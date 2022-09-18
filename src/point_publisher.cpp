#include <ros/ros.h>
#include "nav_cctv/Point.h"
#include "nav_cctv/Locations.h"
#include "nav_cctv/Coordinate.h"
#include <vector>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"point_publisher");
    ros::NodeHandle nh;
    ros::Publisher point_pub = nh.advertise<nav_cctv::Point>("points",1);
    ros::Rate loop_rate(10);
    nav_cctv::Point pub_loc;
    pub_loc.x = 150;
    pub_loc.y = 100;

    int count=0;
    while (ros::ok())
    {
        pub_loc.msg_seq = count;
        ROS_INFO("msg_seq = %d",count);
        point_pub.publish(pub_loc);
        if(pub_loc.x<200 && pub_loc.y == 100) pub_loc.x += 1;
        else if(pub_loc.x == 200 && pub_loc.y<140) pub_loc.y += 1;
        else if(pub_loc.x>150 && pub_loc.y == 140) pub_loc.x -= 1;
        else if(pub_loc.x ==150 && pub_loc.y>100) pub_loc.y -= 1;
        ++count;
        loop_rate.sleep();

    }
    

    return 0;
}