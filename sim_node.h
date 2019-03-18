#ifndef ROBORTS_SIM_SIM_NODE_H
#define ROBORTS_SIM_SIM_NODE_H

#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>

#include <thread>
#include <mutex>
//#include <Eigen/Dense>

#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "roborts_msgs/ShootCmdSim.h"
#include <ros/ros.h>

#include "roborts_sim/CheckBullet.h"
#include "roborts_sim/ReloadCmd.h"
#include "roborts_sim/ShootCmd.h"

#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"

#include "roborts_msgs/TwistAccel.h"
#include "roborts_msgs/GimbalMode.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"

#define THREAD_NUM 4 // ROS SPIN THREAD NUM
namespace roborts_sim {

// base types
// Use eigen3 as base data structure
//using Vec3d = Eigen::Vector3d;

enum Color {red, blue};

Color StrToColor(std::string color) {
  if (color == "red") {
    return red;
  } else {
    return blue;
  }
}

struct RobotInfo {
  RobotInfo(std::string name, Color color, int ammo, int hp):
    name(name),
    color(color),
    ammo(ammo),
    hp(hp)
  {};

  // pose
  geometry_msgs::PoseWithCovariance pose;
  // ammonation
  int ammo;
  // hit points
  int hp;
  // name
  std::string name;
  // color
  Color color;
};

// todo: move SimMap out of header file
class SimMap {
  public:
    SimMap(){}
    SimMap(nav_msgs::OccupancyGrid &map_msg):
      map_msg_(map_msg)
    {
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

    bool hasLineOfSight(double x1, double y1, double x2, double y2, std::vector<geometry_msgs::PoseStamped>& path) {
      ROS_INFO("check line of sight between (%.4f, %.4f) and (%.4f, %.4f)", x1, y1, x2, y2);
      double delta_x = x2 - x1;
      double delta_y = y2 - y1;
      int step_count = static_cast<int>(std::max(std::abs(delta_x) / this->scale_,
        std::abs(delta_y)  / this->scale_));
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

        int index_x = static_cast<int>(pos_x/this->scale_);
        int index_y = static_cast<int>(pos_y/this->scale_);
        ROS_INFO("check %d, %d", index_x, index_y);
        // todo: should also have checked if there is another robot in the way, in which case the 
        // robot in between of the attacker and the target should be shot.
        if (occ_cells_[index_x+index_y*this->size_x_]) {
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
  

class SimNode {
  public:
    SimNode(std::string name);

  private:
    bool Init();
    bool GetStaticMap();
    //void StartSim();


    void PoseCallback(const nav_msgs::Odometry::ConstPtr &pose_msg, const int robot_index);
    // todo to be added after gimbal simulation is added to gazebo
    void GimbalAngleCtrlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg);
    bool SetGimbalModeService(roborts_msgs::GimbalMode::Request &req,
                                  roborts_msgs::GimbalMode::Response &res);
    bool CtrlFricWheelService(roborts_msgs::FricWhl::Request &req,
                                  roborts_msgs::FricWhl::Response &res);
    bool CtrlShootService(roborts_msgs::ShootCmd::Request &req,
                              roborts_msgs::ShootCmd::Response &res);
    void PublishPath(const std::vector<geometry_msgs::PoseStamped> &path);
    bool TryShoot(int robot1, int robot2);
    bool TryReload(int robot);

    bool ShootCmd(roborts_msgs::ShootCmdSim::Request &req,
                  roborts_msgs::ShootCmdSim::Response &res);
    bool ReloadCmd(roborts_sim::ReloadCmd::Request &req, 
                  roborts_sim::ReloadCmd::Response & res);
                  
    void AmmoDown(int robot, int num);
    void HpDown(int robot, int damage);
    bool CheckBullet(roborts_sim::CheckBullet::Request &req,roborts_sim::CheckBullet::Response &res);
    void CountDown();
  private:
    //ROS Node handle
    ros::NodeHandle nh_;

    
    /**
     ******* ROS Subscriber *******
     */
    // listen to gazebo for the real poses
    std::vector<ros::Subscriber> gazebo_real_pose_sub_;
    // listen to gimbal executor from decision node and detection node
    std::vector<ros::Subscriber> ros_gimbal_angle_sub_;
    
    // to be removed
    // ros::Subscriber sub_r1_;
    // ros::Subscriber sub_r2_;
    // ros::Subscriber sub_r3_;
    // ros::Subscriber sub_r4_;

    /**
     ******* ROS Publisher *******
     */
    // publish visualization for the LOS
    ros::Publisher path_pub_;

    /**
     ******* ROS Service *******
     */
    // static map client, needed for LOS test
    ros::ServiceClient ros_static_map_srv_;
    // ros service server for gimbal mode set
    std::vector<ros::ServiceServer> ros_gimbal_mode_srv_;
    // ros service server for friction wheel control
    std::vector<ros::ServiceServer> ros_ctrl_fric_wheel_srv_;
    // ros service server for gimbal shoot control
    std::vector<ros::ServiceServer> ros_ctrl_shoot_srv_;
    

    ros::ServiceServer shoot_srv_;
    ros::ServiceServer check_bullet_srv_;
    //reload srv
    ros::ServiceServer reload_srv_;

    /**
     ******* Status *******
     */ 
    bool first_map_received_ = false;
    bool is_showing_los_ = false;

    /**
     ******* Data *******
     */ 
    std::vector<RobotInfo> robot_info_;
    SimMap map_;
    nav_msgs::Path path_;

    /**
     ******* Misc *******
     */ 
    // unsure if the rest is necessary; also, please avoid 
    // using array and stick to std:vector.
    //time
    int reloadTime[4] = {0,0,0,0};
    int m = 3;
    int s = 0;
};


} // roborts_sim

#endif // ROBORTS_SIM_SIM_NODE_H
