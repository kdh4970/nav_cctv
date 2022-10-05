#include "cctv_layer/cctv_layer.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include "/home/capstone/catkin_ws/src/navigation/move_base/src/global_multi_point.cpp"

extern std::vector<int> received_point_x;
extern std::vector<int> received_point_y;
extern int received_point_msg_seq;

PLUGINLIB_EXPORT_CLASS(cctv_layer_namespace::CctvLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;



namespace cctv_layer_namespace
{

CctvLayer::CctvLayer() {}


void CctvLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();


  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &CctvLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void CctvLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void CctvLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void CctvLayer::clearPastcost(std::vector<char> &pastcost)
{
  for(int i = 0; i < pastcost.size();i++) setCost(past_x[i], past_y[i], pastcost[i]);
}

void CctvLayer::transformCoordinate(std::vector<int> &yolo_x,std::vector<int> &yolo_y,std::vector<int> &costmap_x,std::vector<int> &costmap_y)
{
  for (int i=0;i<yolo_x.size();i++)
  {
    costmap_x.push_back(yolo_map_origin_x + round((yolo_x[i] - 174) * yolo_to_costmap_scale));
    costmap_y.push_back(yolo_map_origin_y - round((yolo_y[i] - 87) * yolo_to_costmap_scale));
  }
}

void CctvLayer::makeCircle(int point_x, int point_y,std::vector<int> &circle_x, std::vector<int> &circle_y)
{
  int radius = 3;

  // Calculate Circle point
  for (int i = -radius ; i <= radius ; i++)
  {
    for(int j = -radius ; j <= radius ; j++)
    {
      if(i*i + j*j <= radius*radius)
      {
        circle_x.push_back(point_x+i);
        circle_y.push_back(point_y+j);
      }
    }
  }
}


void CctvLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  double mark_x, mark_y;
  std::vector<int> result_x,result_y;

  // Change from received yolo point coordinate to costmap coordinate
  transformCoordinate(received_point_x,received_point_y,result_x,result_y);
  
  // Clear point which was marked in past loop 
  for(int i=0; i<past_x.size();i++) clearPastcost(pastcost);
  
  // Reset past location vector
  std::vector<int>().swap(past_x);
  std::vector<int>().swap(past_y);
  std::vector<char>().swap(pastcost);


  for(int i=0; i<result_x.size();i++)
  {
    std::vector<int> circle_x,circle_y;
    makeCircle(result_x[i],result_y[i],circle_x,circle_y);
    for(int j=0; j<circle_x.size();j++)
    {
      // Save cost
      pastcost.push_back(getCost(circle_x[j],circle_y[j]));

      // Marking
      unsigned int mx = circle_x[j];
      unsigned int my = circle_y[j];
      mapToWorld(mx, my, mark_x, mark_y);
      setCost(mx, my, LETHAL_OBSTACLE);
      
      *min_x = std::min(*min_x, mark_x);
      *min_y = std::min(*min_y, mark_y);
      *max_x = std::max(*max_x, mark_x);
      *max_y = std::max(*max_y, mark_y);
    
      // Allocate new past location
      past_x.push_back(circle_x[j]);
      past_y.push_back(circle_y[j]);
    }
  }

  // for(int i=0; i<result_x.size();i++)
  // {
    
  //   // Save cost
  //   pastcost.push_back(getCost(result_x[i],result_y[i]));

  //   // Marking
  //   unsigned int mx = result_x[i];
  //   unsigned int my = result_y[i];
  //   mapToWorld(mx, my, mark_x, mark_y);
  //   setCost(mx, my, LETHAL_OBSTACLE);
      
  //   *min_x = std::min(*min_x, mark_x);
  //   *min_y = std::min(*min_y, mark_y);
  //   *max_x = std::max(*max_x, mark_x);
  //   *max_y = std::max(*max_y, mark_y);
    
  //   // Allocate new past location
  //   past_x.push_back(result_x[i]);
  //   past_y.push_back(result_y[i]);

  // }

  

}

void CctvLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);     

      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
  

  // set cost to master costmap directly, but it doesn't inflate
  //master_grid.setCost(received_point_x, received_point_y, LETHAL_OBSTACLE);
  //master_grid.setCost(past_x, past_y, 0);
  //past_x = received_point_x, past_y = received_point_y;
}


} // end namespace