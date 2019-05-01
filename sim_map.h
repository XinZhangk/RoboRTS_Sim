#ifndef ROBORTS_SIM_SIM_MAP_H
#define ROBORTS_SIM_SIM_MAP_H

#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>


namespace roborts_sim {

class SimMap {
public:
  SimMap() {}

  SimMap(nav_msgs::OccupancyGrid &map_msg) :
    map_msg_(map_msg) {
    this->origin_x_ = map_msg.info.origin.position.x;
    this->origin_y_ = map_msg.info.origin.position.y;
    this->scale_ = map_msg.info.resolution;
    this->size_x_ = map_msg.info.width;
    this->size_y_ = map_msg.info.height;
    this->occ_cells_.resize(this->size_x_ * this->size_y_);

    for (int i = 0; i < this->size_x_ * this->size_y_; i++) {
      auto tmp_msg = static_cast<int>(map_msg.data[i]);
      if (tmp_msg == 100) {
        this->occ_cells_[i] = true;
      } else {
        this->occ_cells_[i] = false;
      }
    }
  }

  bool hasLineOfSight(double x1, double y1, double x2, double y2, std::vector <geometry_msgs::PoseStamped> &path) {
    ROS_INFO("check line of sight between (%.4f, %.4f) and (%.4f, %.4f)", x1, y1, x2, y2);
    if (x1==x2 && x2==y2)
    {
      ROS_WARN("checking same location");
      return false;
    }
    double delta_x = x2 - x1;
    double delta_y = y2 - y1;
    int step_count = static_cast<int>(std::max(std::abs(delta_x) / this->scale_,
                                               std::abs(delta_y) / this->scale_));
    double step_x = delta_x / step_count;
    double step_y = delta_y / step_count;
    for (int i = 0; i < step_count; i++) {
      double pos_x = x1 + step_x * i;
      double pos_y = y1 + step_y * i;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = pos_x;
      pose.pose.position.y = pos_y;
      pose.pose.position.z = 0;
      path.push_back(pose);

      int index_x = static_cast<int>(pos_x / this->scale_);
      int index_y = static_cast<int>(pos_y / this->scale_);
      //ROS_INFO("check %d, %d", index_x, index_y);
      // todo: should also have checked if there is another robot in the way, in which case the
      // robot in between of the attacker and the target should be shot.
      if (occ_cells_[index_x + index_y * this->size_x_]) {
        return false;
      }
    }
    return true;
  }

private:
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
};
}


#endif //ROBORTS_SIM_SIM_MAP_H