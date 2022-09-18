#include <ros/ros.h>
#include "nav_cctv/MultiPoint.h"
#include <vector>


int main(int argc, char **argv)
{
    ros::init(argc,argv,"point_publisher");
    ros::NodeHandle nh;
    ros::Publisher point_pub = nh.advertise<nav_cctv::MultiPoint>("points",1);
    ros::Rate loop_rate(2);
    nav_cctv::MultiPoint pub_loc;
    int ds=0;

    int count=0;
    while (ros::ok())
    {
        pub_loc.msg_seq = count;
        std::vector<int16_t> temp_pub_x;
        std::vector<int16_t> temp_pub_y;

        temp_pub_x.push_back(180+ds);
        temp_pub_y.push_back(180+ds);

        temp_pub_x.push_back(200);
        temp_pub_y.push_back(145+ds);
    
        temp_pub_x.push_back(240-ds);
        temp_pub_y.push_back(220-ds);

        temp_pub_x.push_back(175);
        temp_pub_y.push_back(170);

        temp_pub_x.push_back(170);
        temp_pub_y.push_back(165);

        pub_loc.x = temp_pub_x;
        pub_loc.y = temp_pub_y;
        //ROS_INFO("x >> %d, y >> %d //msg_seq = %d",pub_loc.x,pub_loc.y,count);
        
        point_pub.publish(pub_loc);
        ++count;
        ++ds;
        if(ds>70) ds=0;
        loop_rate.sleep();

    }
    

    return 0;
}