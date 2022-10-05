#ifndef CCTV_LAYER_H_
#define CCTV_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace cctv_layer_namespace
{

class CctvLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  CctvLayer();
  int yolo_map_origin_x = 990;
  int yolo_map_origin_y = 1984-855;
  int yolo_to_costmap_scale = 0.1997;
  std::vector<int> past_x ;
  std::vector<int> past_y ;
  std::vector<char> pastcost;

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }
  virtual void matchSize();
  
private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  void clearPastcost(std::vector<char> &pastcost);
  void transformCoordinate(std::vector<int> &yolo_x,std::vector<int> &yolo_y,std::vector<int> &costmap_x,std::vector<int> &costmap_y);
  void markCircle(int point_x, int point_y,double* min_x,double* min_y, double* max_x, double* max_y);
};
}
#endif