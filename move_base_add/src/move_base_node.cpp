/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <move_base/move_base.h>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/String.h"
#include "MultiPoint.h"
#include "global_multi_point.cpp"

extern std::vector<int> received_point_x;
extern std::vector<int> received_point_y;
extern int received_point_msg_seq;
void PointCallback(const nav_cctv::MultiPointConstPtr&Topic_point);

int main(int argc, char** argv){
  ros::init(argc, argv, "move_base_node");
  ros::Subscriber point_listen_;
  ros::NodeHandle point_listen_nh_;
  point_listen_ = point_listen_nh_.subscribe("points",5,PointCallback);



  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);

  move_base::MoveBase move_base( buffer );

  

  
  //ros::MultiThreadedSpinner s;
  ros::spin();

  return(0);
}

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
