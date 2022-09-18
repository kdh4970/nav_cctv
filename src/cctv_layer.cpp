#include "cctv_layer/cctv_layer.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "/home/capstone/catkin_ws/src/navigation/move_base/src/global_multi_point.cpp"

extern std::vector<int> received_point_x;
extern std::vector<int> received_point_y;
extern int received_point_msg_seq;
std::vector<double> past_x ;
std::vector<double> past_y ;
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

void CctvLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  std::vector<char> pastcost;
  
  //double mark_x = (double)received_point_x , mark_y= (double)received_point_y;
  double mark_x, mark_y;

  
  for(int i=0; i<received_point_x.size();i++)
  {
    // Clearing
    for(int i = 0; i < pastcost.size();i++) 
    {
      if(past_x.size()==0 || past_y.size()==0) // Ignore first Clearing
        break;
      setCost(past_x[i], past_y[i], pastcost[i]);
    }
    
    // Save cost
    pastcost.push_back(getCost(received_point_x[i],received_point_y[i]));

    // Marking
    unsigned int mx = received_point_x[i];
    unsigned int my = received_point_y[i];
    mapToWorld(mx, my, mark_x, mark_y);
    setCost(mx, my, LETHAL_OBSTACLE);

    *min_x = std::min(*min_x, mark_x);
    *min_y = std::min(*min_y, mark_y);
    *max_x = std::max(*max_x, mark_x);
    *max_y = std::max(*max_y, mark_y);
  }



  // Reset past location vector
  std::vector<double>().swap(past_x);
  std::vector<double>().swap(past_y);

  // Allocate new past location
  for(int i = 0; i< received_point_x.size();i++)
  {
    past_x.push_back(received_point_x[i]);
    past_y.push_back(received_point_y[i]);
  }
  

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