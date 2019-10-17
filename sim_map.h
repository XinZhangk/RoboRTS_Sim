#ifndef ROBORTS_SIM_SIM_MAP_H
#define ROBORTS_SIM_SIM_MAP_H

#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include "roborts_msgs/SimHasLoS.h"
#include <nav_msgs/Path.h>
#include <mutex>

namespace roborts_sim {

class SimMap {
public:
  SimMap();
  void Init(nav_msgs::OccupancyGrid &map_msg);
  bool hasLineOfSight(double x1, double y1, double x2, double y2);
  
private:
  std::vector <geometry_msgs::PoseStamped> path_;
  std::mutex mutex_;
  nav_msgs::OccupancyGrid map_msg_;
  double origin_x_ = 0, origin_y_ = 0;
  /**
    * @brief Map scale (m/cell)
    */
  double scale_ = 0;
  /**
   * @brief Map dimensions (number of cells)
   */
  int size_x_ = 0, size_y_ = 0;
  std::vector<bool> occ_cells_;
  ros::ServiceServer has_los_srv_;
  // publish visualization for the LOS
  ros::Publisher path_pub_;
  bool CtrlLoSService(roborts_msgs::SimHasLoS::Request& req, 
                      roborts_msgs::SimHasLoS::Response& res);
  void PublishPath();
  
};
}


#endif //ROBORTS_SIM_SIM_MAP_H
