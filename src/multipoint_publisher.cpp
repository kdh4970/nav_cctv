#include <ros/ros.h>
#include "nav_cctv/MultiPoint.h"
#include <vector>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"point_publisher");
    ros::NodeHandle nh;
    ros::Publisher point_pub = nh.advertise<nav_cctv::MultiPoint>("points",1);
    ros::Rate loop_rate(4);
    nav_cctv::MultiPoint pub_loc;


    int count=0;
    while (ros::ok())
    {
        pub_loc.msg_seq = count;
        std::vector<int16_t> temp_pub_x;
        std::vector<int16_t> temp_pub_y;

        temp_pub_x.push_back(80+count);
        temp_pub_y.push_back(80+count);

        temp_pub_x.push_back(100);
        temp_pub_y.push_back(80+count);
    
        temp_pub_x.push_back(140-count);
        temp_pub_y.push_back(100-count);

        pub_loc.x = temp_pub_x;
        pub_loc.y = temp_pub_y;
        //ROS_INFO("x >> %d, y >> %d //msg_seq = %d",pub_loc.x,pub_loc.y,count);
        
        point_pub.publish(pub_loc);
        ++count;
        loop_rate.sleep();

    }
    

    return 0;
}