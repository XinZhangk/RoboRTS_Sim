#ifndef ROBORTS_SIM_SIM_NODE_H
#define ROBORTS_SIM_SIM_NODE_H

#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include <thread>
#include <mutex>
#include <algorithm>
#include <chrono>
//#include <Eigen/Dense>

#include "sim_map.h"

#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/PoseStamped.h>
#include "roborts_msgs/SimShootCmd.h"
#include <ros/ros.h>
#include <tf/tf.h>

// TODO: Use another header file to include such a bunch of protocol head files
#include "roborts_msgs/SimCheckBullet.h"
#include "roborts_msgs/SimReloadCmd.h"
#include "roborts_msgs/SimCountdown.h"

#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"

#include "roborts_msgs/TwistAccel.h"
#include "roborts_msgs/GimbalMode.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/RobotBonus.h"
#include "roborts_msgs/BonusStatus.h"
#include "roborts_msgs/SupplierStatus.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/GameSurvivor.h"
#include "roborts_msgs/ProjectileSupply.h"

#define PI 3.1416
#define THREAD_NUM 8 // ROS SPIN THREAD NUM
namespace roborts_sim {

// base types
// Use eigen3 as base data structure
//using Vec3d = Eigen::Vector3d;

// Define game environment constants
const int INITIAL_HP = 2000;
const int DAMAGE_PER_BULLET = 50;
const int INITIAL_BULLET_AMOUNT = 40;
const int ROBOT_NUM = 4;
// barrel cooling relevant
const int PROJECTILE_SPEED = 25;
const int BARREL_HEAT_LIMIT = 360;
const int BARREL_HEAT_UPPERBOUND = 720;
const int BARREL_COOLING_RATE = 24;
const int BARREL_SETTLE_FEQ = 10;

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
    hp(hp),
    barrel_heat(0){};

  // pose
  geometry_msgs::PoseWithCovariance pose;
  // ammunition
  int ammo;
  // hit points
  int hp;
  // name
  std::string name;
  // color
  Color color;
  // reload time
  int reload_time;
  // barrel heat
  int barrel_heat;
  //bonus
  bool bonus = false;
  //bonus time
  int buff_time = 0;
};

class SimNode {
  public:
    SimNode(std::string name);

  private:
    // Initialization Relevant Methods
    bool Init();
    bool GetStaticMap();
    //void StartSim();

    void InitializeRobotInfo();

    // Game information Update Methods
    void CountDown();
    void GameCountDown();
    void resetReload();
    void gameEnd(const ros::TimerEvent&, int i);
    void AmmoDown(int robot, int num);
    void HpDown(int robot, int damage, int damage_type, int damage_source = 0);

    // Shooting Service Methods
    bool ShootCmd(roborts_msgs::SimShootCmd::Request &req, roborts_msgs::SimShootCmd::Response &res);
    bool TryShoot(int robot1, int robot2);

    // Overheating rule functions
    void AddBarrelHeat(int robot);
    void PublishRobotHeat(int robot);
    void SettleRobotHeat(int robot);
    int ComputeBarrelDamage(int robot);

    // Reloading Methods
    void ReloadCallingCallback(const roborts_msgs::ProjectileSupply::ConstPtr &msg, const int robot);
    bool TryReload(int robot);
    //bool ReloadDetector(bool red);

    //Buff Zone
    bool BuffzoneDetector(bool red);
    void TryRedBuff();
    void TryBlueBuff();
    void resetBufftime();
    void resetBuff(bool red);

    // Check bullet service methods
    bool CheckBullet(roborts_msgs::SimCheckBullet::Request &req,roborts_msgs::SimCheckBullet::Response &res);

    // Robot Status Publisher
    void ExecutePublishLoop();
//    void StartThread();
//    void StopThread();
//    void ExecuteLoop(int robot);
    void PublishRobotStatus(int robot);
    void PublishGameStatus(int robot);
    void PublishGameSurvivor();
    void PublishBonus(int robot);
    void PublishBonusStatus(int robot);

    // Uncategorized
    bool SetGimbalModeService(roborts_msgs::GimbalMode::Request &req,
                              roborts_msgs::GimbalMode::Response &res);
    bool CtrlFricWheelService(roborts_msgs::FricWhl::Request &req,
                              roborts_msgs::FricWhl::Response &res);

    // updated version for shoot service
    bool CtrlShootService(roborts_msgs::ShootCmd::Request &req,
                          roborts_msgs::ShootCmd::Response &res, const int robot_index);
    double GetYaw(geometry_msgs::Quaternion orientation);

    void PoseCallback(const nav_msgs::Odometry::ConstPtr &pose_msg, const int robot_index);
    // todo to be added after gimbal simulation is added to gazebo
    void GimbalAngleCtrlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg);


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

    /**
     ******* ROS Publisher *******
     */

    std::vector<ros::Publisher> ros_countdown_pub_;
    std::vector<ros::Publisher> ros_robot_status_pub_;
    std::vector<ros::Publisher> ros_robot_damage_pub_;
    std::vector<ros::Publisher> ros_robot_heat_pub_;
    std::vector<ros::Publisher> ros_robot_bonus_pub_;
    std::vector<ros::Publisher> ros_robot_bonus_status_pub_;
    std::vector<ros::Publisher> ros_robot_game_status_pub_;
    std::vector<ros::Publisher> ros_robot_game_survivor_pub_;
    std::vector<ros::Publisher> ros_robot_supplier_status_pub_;

    /**
     ******* ROS Subscriber *******
     */
    std::vector<ros::Subscriber> ros_robot_reload_calling_sub_;

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
    std::vector<ros::ServiceServer> reload_srv_;

    /**
     ******* Status *******
     */
    bool first_map_received_ = false;
    bool is_showing_los_ = false;
    // remain time
    int remaining_time = 305;
    int red_bonus_time = 0;
    int blue_bonus_time = 0;
    /**
     ******* Data *******
     */
    std::vector<RobotInfo> robot_info_;
    SimMap map_;
    

    /**
     ******* Thread *****
     */
    std::vector<std::thread> robot_status_publisher_thread_;
    std::vector<std::thread> robot_heat_publisher_thread_;
    std::vector<std::thread> robot_medium_thread_;

    /**
     ******* Misc *******
     */
    // unsure if the rest is necessary; also, please avoid
    // using array and stick to std:vector.
    //time
    //ros::Publisher cd;
    //std::vector<int> reloadTime;
    ros::Timer reload_timer_;
    std::vector<ros::Timer> countdown_timer_;
    std::vector<ros::Timer> barrel_heat_timer_;
    std::mutex mutex_;

    //bonus msg
    roborts_msgs::BonusStatus bonus_status_;
};


} // roborts_sim

#endif // ROBORTS_SIM_SIM_NODE_H
