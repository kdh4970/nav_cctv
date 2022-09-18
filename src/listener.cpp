#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_cctv/MultiPoint.h"
#include "/home/capstone/catkin_ws/src/navigation/move_base/src/global_multi_point.cpp"


extern std::vector<int> received_point_x;
extern std::vector<int> received_point_y;
extern int received_point_msg_seq;

void PointCallback(const nav_cctv::MultiPointConstPtr&Topic_point)
{
    //ROS_INFO("Point is %d, seq is %d",Topic_point->location,Topic_point->msg_seq);
    int loc_size = Topic_point->x.size();
    std::vector<int> temp_x;
    std::vector<int> temp_y;
    for(int i =0 ; i<loc_size ; i++)
    {
        temp_x.push_back(Topic_point->x[i]);
        temp_y.push_back(Topic_point->y[i]);
    }
    received_point_x = temp_x;
    received_point_y = temp_y;
    
    for(int i =0; i<received_point_x.size();i++)
    {
        ROS_INFO("x%d,y%d = %d,%d",i,i,received_point_x[i],received_point_y[i]);
    }
    received_point_msg_seq = Topic_point->msg_seq;

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"listener");
    ros::Subscriber point_listen_;
    ros::NodeHandle point_listen_nh_;
    point_listen_ = point_listen_nh_.subscribe("points",5,PointCallback);
    

    ros::spin();    


}


