#include "sim_node.h"

namespace roborts_sim{

SimNode::SimNode(std::string name) {
  Init();
  ROS_INFO("Simulation node initialized");
  //StartSim();

}

bool SimNode::Init() {
  // request the static map
  GetStaticMap();
  // initialize robot informatoin
  InitializeRobotInfo();


  // create subscribers and services
  for (unsigned i = 0; i < robot_info_.size(); i++) {
    // initialize subscribers and service servers
    std::string pose_topic                = "/" + robot_info_[i].name + "/" + "gazebo_robot_pose";
    std::string gimbal_angle_topic        = "/" + robot_info_[i].name + "/" + "cmd_gimbal_angle";
    std::string gimbal_mode_service_name  = "/" + robot_info_[i].name + "/" + "set_gimbal_mode";
    std::string fric_wheel_service_name   = "/" + robot_info_[i].name + "/" + "cmd_fric_wheel";
    std::string gimbal_shoot_service_name = "/" + robot_info_[i].name + "/" + "cmd_shoot";
    std::string countdown_topic_name      = "/" + robot_info_[i].name + "/" + "countdown";
    std::string robot_status_topic_name   = "/" + robot_info_[i].name + "/" + "robot_status";
    std::string robot_damage_topic_name   = "/" + robot_info_[i].name + "/" + "robot_damage";
    std::string robot_heat_topic_name     = "/" + robot_info_[i].name + "/" + "robot_heat";
    // fill up the vectors
    gazebo_real_pose_sub_.push_back(nh_.subscribe<nav_msgs::Odometry>(pose_topic, 100, boost::bind(&SimNode::PoseCallback,this,_1,i)));
    ros_gimbal_angle_sub_.push_back(nh_.subscribe(gimbal_angle_topic, 1, &SimNode::GimbalAngleCtrlCallback, this));
    ros_gimbal_mode_srv_.push_back(nh_.advertiseService(gimbal_mode_service_name, &SimNode::SetGimbalModeService, this));
    ros_ctrl_fric_wheel_srv_.push_back(nh_.advertiseService(fric_wheel_service_name, &SimNode::CtrlFricWheelService, this));
    ros_ctrl_shoot_srv_.push_back(nh_.advertiseService(gimbal_shoot_service_name, &SimNode::CtrlShootService, this));
    ros_countdown_pub_.push_back(nh_.advertise<roborts_sim::Countdown>(countdown_topic_name, 1000));
    ros_robot_status_pub_.push_back(nh_.advertise<roborts_msgs::RobotStatus>(robot_status_topic_name, 30));
    ros_robot_damage_pub_.push_back(nh_.advertise<roborts_msgs::RobotStatus>(robot_damage_topic_name, 30));
    ros_robot_heat_pub_.push_back(nh_.advertise<roborts_msgs::RobotStatus>(robot_heat_topic_name, 30));
  }

  // for visualization
  path_pub_ = nh_.advertise<nav_msgs::Path>("los_path", 10);


  // following services are deprecated; we should stick to the protocols used
  // on real robots, rather than define our own
  shoot_srv_ = nh_.advertiseService("shoot", &SimNode::ShootCmd, this);
  reload_srv_ = nh_.advertiseService("reload", &SimNode::ReloadCmd, this);

  // Advertise Check Bullet service
  check_bullet_srv_ = nh_.advertiseService("check_bullet",&SimNode::CheckBullet,this);
  //ROS_INFO("check bullet service ready");

// can't time be measured better than reading the machine time? Remember the wall time is not necessary
// the simulation time! In simulation time may "pass" much slower due to performance reasons
  //cd = nh_.advertise<roborts_sim::Countdown>("countdown", 1000);

  // reloadTime.push_back(0);
  // reloadTime.push_back(0);
  // reloadTime.push_back(0);
  // reloadTime.push_back(0);

  CountDown();
  //std::thread t(&SimNode::CountDown,this);
  //t.join();
  return true;
}

void SimNode::InitializeRobotInfo(){
  int hp, ammo_count;
  std::vector<std::string> name_list = {"r1", "r2", "r3", "r4"};
  std::vector<std::string> color_list = {"red", "red", "blue", "blue"};
  nh_.param<int>("/hp", hp, INITIAL_HP);
  nh_.param<int>("/ammo", ammo_count, INITIAL_BULLET_AMOUNT);
  nh_.param<std::vector<std::string>>("/name_list", name_list);
  nh_.param<std::vector<std::string>>("/color_list", color_list);

  for(unsigned i = 0; i < name_list.size(); i++){
    Color color = StrToColor(color_list[i]);
    robot_info_.push_back(RobotInfo(name_list[i], color, ammo_count, hp));
  }
}

// Check Bullet Service
// I think the caller of this service should be the robot, in our simulation.
// The rationale is that the whole weaponry system is controlled by the simulation node, virtually.
// Although this is counter-intuitive as the robot is supposed to have full control over the bullets they shoot out,
// I think it is easier programming-wise to consolidate all the combat information in the simulation node
bool SimNode::CheckBullet(roborts_sim::CheckBullet::Request &req,roborts_sim::CheckBullet::Response &res){
  res.remaining_bullet = robot_info_[req.robot_id-1].ammo;
  ROS_INFO("request: robot %d remaining bullet",req.robot_id);
  ROS_INFO("response: remaining bullet = %d",res.remaining_bullet);
  return true;
}

void SimNode::StartThread() {
  for(unsigned i = 0; i < ROBOT_NUM; i++) {
    robot_medium_thread_.push_back(std::thread(&SimNode::ExecuteLoop, this, i));
  }
}

void SimNode::StopThread() {
  for(unsigned i = 0; i < ROBOT_NUM; i++) {
    if (robot_medium_thread_[i].joinable()) {
      robot_medium_thread_[i].join();
    }
  }
}

void SimNode::ExecuteLoop(int robot) {
  ros::Rate r(10);
  while (ros::ok()) {
    PublishRobotStatus(robot);
    PublishRobotHeat(robot);
    SettleRobotHeat(robot);
    ros::spinOnce();
    r.sleep();
  }
}

void SimNode::PublishRobotStatus(int robot) {
  roborts_msgs::RobotStatus robot_status;
  // transform literal robot name to numeric id
  if (robot == 0) {
    robot_status.id = 1;
    robot_status.remain_hp = robot_info_[0].hp;
    ros_robot_status_pub_[0].publish(robot_status);
  } else if (robot == 1) {
    robot_status.id = 2;
    robot_status.remain_hp = robot_info_[1].hp;
    ros_robot_status_pub_[1].publish(robot_status);
  } else if (robot == 2) {
    robot_status.id = 13;
    robot_status.remain_hp = robot_info_[2].hp;
    ros_robot_status_pub_[2].publish(robot_status);
  } else if (robot == 3) {
    robot_status.id = 14;
    robot_status.remain_hp = robot_info_[3].hp;
    ros_robot_status_pub_[3].publish(robot_status);
  } else {
    ROS_WARN("For AI challenge, please set robot id to Blue3/4 or Red3/4 in the referee system main control module");
    return;
  }
}

void SimNode::PublishRobotHeat(int robot) {
  roborts_msgs::RobotHeat robot_heat;
  robot_heat.shooter_heat = robot_info_[robot-1].barrel_heat;
  ros_robot_heat_pub_[robot-1].publish(robot_heat);
}

bool SimNode::GetStaticMap(){
  ros_static_map_srv_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
  ros::service::waitForService("/static_map", -1);
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;
  if(ros_static_map_srv_.call(req,res)) {
    ROS_INFO("Received Static Map");
    map_ = SimMap(res.map);
    first_map_received_ = true;
    return true;
  } else{
    ROS_ERROR("Get static map failed");
    return false;
  }
}

void SimNode::PublishPath(const std::vector<geometry_msgs::PoseStamped> &path) {
  path_.poses = path;
  path_.header.frame_id = "map";
  path_pub_.publish(path_);
}

void SimNode::AmmoDown(int robot, int num) {
  {
    std::lock_guard <std::mutex> guard(mutex_);
    robot_info_[robot - 1].ammo -= num;
  }
  AddBarrelHeat(robot);
}

void SimNode::AddBarrelHeat(int robot) {
  int current_barrel_heat = robot_info_[robot-1].barrel_heat + PROJECTILE_SPEED;
  if (current_barrel_heat <= BARREL_HEAT_UPPERBOUND) {
    std::lock_guard<std::mutex> guard(mutex_);
    robot_info_[robot-1].barrel_heat = current_barrel_heat;
  } else {
    int overheat_damage = (current_barrel_heat - BARREL_HEAT_UPPERBOUND) * 40;
    HpDown(robot, overheat_damage, 2);
    {
      std::lock_guard <std::mutex> guard(mutex_);
      robot_info_[robot - 1].barrel_heat = BARREL_HEAT_UPPERBOUND;
    }
  }
}

void SimNode::SettleRobotHeat(int robot) {
  std::lock_guard <std::mutex> guard(mutex_);
  int barrel_heat = robot_info_[robot-1].barrel_heat;
  int robot_hp = robot_info_[robot-1].hp;
  if (barrel_heat > 720) {
    ROS_WARN("There must be a synchronization problem.");
  }
  if (robot_hp >= 400) {
    robot_info_[robot-1].barrel_heat -= static_cast<int>(BARREL_COOLING_RATE / 2);
  } else {
    robot_info_[robot-1].barrel_heat -= BARREL_COOLING_RATE;
  }
  HpDown(robot, ComputeBarrelDamage(barrel_heat), 2);
}

int SimNode::ComputeBarrelDamage(int barrel_heat) {
  if (barrel_heat >= 720) {
    return (barrel_heat - BARREL_HEAT_UPPERBOUND) * 40;
  } else if (barrel_heat > 360) {
    return (barrel_heat - BARREL_HEAT_LIMIT) * 4;
  } else {
    return 0;
  }
}

void SimNode::HpDown(int robot, int damage, int damage_type){
  {
    std::lock_guard <std::mutex> guard(mutex_);
    robot_info_[robot-1].hp -= damage;
  }
  ROS_INFO("Robot %d lost %d hit points", robot, damage);
  roborts_msgs::RobotDamage robot_damage;
  robot_damage.damage_type = damage_type;
  ros_robot_status_pub_[robot-1].publish(robot_damage);
  if (robot_info_[robot-1].hp < 0)
  {
    roborts_sim::Countdown cdm;
    cdm.gameState = "Countdown ends!";
    ROS_INFO("robot %d is dead", robot);
    ros_countdown_pub_[robot-1].publish(cdm);
  }
}

bool SimNode::TryShoot(int robot1, int robot2) {
  ROS_INFO("Robot %d tries to shoot Robot %d", robot1, robot2);
  std::vector<geometry_msgs::PoseStamped> currentPath;
  auto r1_pos = robot_info_[robot1-1].pose.pose.position;
  auto r2_pos = robot_info_[robot2-1].pose.pose.position;
  if (map_.hasLineOfSight(r1_pos.x, r1_pos.y, r2_pos.x, r2_pos.y, currentPath)){
    ROS_INFO("Robot %d and Robot %d can see each other", robot1, robot2);
    if (robot_info_[robot1-1].ammo > 0) {
      HpDown(robot2, DAMAGE_PER_BULLET, 0);
      AmmoDown(robot1, 1);
    } else {
      ROS_WARN("Robot %d has no ammo but tries to shoot.", robot1);
    }
  } else {
    ROS_WARN("Robot %d and Robot %d cannot see each other", robot1, robot2);
  }
  PublishPath(currentPath);
}

bool SimNode::TryReload(int robot){
  ROS_INFO("Robot %d tries reloading.", robot);

  if(robot_info_[robot-1].reload_time > 1){
    ROS_INFO("Robot %d has reloaded for 2 times in 1 minute, failed.", robot);
    return false;
  }else{
    robot_info_[robot-1].reload_time++;
    ROS_INFO("Robot %d has reloaded for %d times in 1 minute, succeed.", robot, robot_info_[robot-1].reload_time);
    sleep(5);
    robot_info_[robot-1].ammo = 50;
    return true;
  }
}


// Use a universal pose callback method
void SimNode::PoseCallback(const nav_msgs::Odometry::ConstPtr &pose_msg, const int robot_index){
  auto position = pose_msg->pose.pose.position;
  auto orientation = pose_msg->pose.pose.orientation;
  robot_info_[robot_index].pose = geometry_msgs::PoseWithCovariance(pose_msg->pose);
  //auto pose = robot_info_[robot_index].pose;
  //ROS_INFO("r[%d] has pose [%.4f ,%.4f ,%.4f]", topic,pose.pose.position.x, pose.pose.position.y, pose.pose.position.z );
}

void SimNode::GimbalAngleCtrlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg){}

bool SimNode::SetGimbalModeService(roborts_msgs::GimbalMode::Request &req,
                                  roborts_msgs::GimbalMode::Response &res){
  return false;
                                  }

bool SimNode::CtrlFricWheelService(roborts_msgs::FricWhl::Request &req,
                                  roborts_msgs::FricWhl::Response &res){
  return false;
                                  }

bool SimNode::CtrlShootService(roborts_msgs::ShootCmd::Request &req,
                              roborts_msgs::ShootCmd::Response &res){
  return false;
}

bool SimNode::ShootCmd(roborts_msgs::ShootCmdSim::Request &req,
                  roborts_msgs::ShootCmdSim::Response &res){
  int robot1 = req.robot;
  int robot2 = req.enemy;
  res.success = TryShoot(robot1, robot2);
  return true;
}

bool SimNode::ReloadCmd(roborts_sim::ReloadCmd::Request &
                  req, roborts_sim::ReloadCmd::Response & res){
  int robot = req.robot;
  res.success = TryReload(robot);
  res.supply_projectile_id = 1;
  res.supply_robot_id = req.robot;
  res.supply_num = 50;
  return true;
}

void SimNode::CountDown(){
  roborts_sim::Countdown cdm;
  cdm.gameState = "Game starts!";
  ROS_INFO("Game starts!");


  // for (auto & pub : ros_countdown_pub_) {
  //   pub.publish(cdm);
  // }
  reload_timer_ = nh_.createTimer(ros::Duration(60), &SimNode::resetReload, this);
  //ros::spinOnce();
  //ros::spin();
  //nh_.createTimer(ros::Duration(120), &SimNode::resetReload, this);
  //ros::spinOnce();
  ros::spinOnce();

    //cd.publish(cdm);
  for (int i = 0; i < ros_countdown_pub_.size(); i ++) {
    ros_countdown_pub_[i].publish(cdm);
    countdown_timer_.push_back(nh_.createTimer(ros::Duration(180), boost::bind(&SimNode::gameEnd, this, _1, i)));
    ros::spinOnce();
    //ros::spin();
  }

}

void SimNode::resetReload(const ros::TimerEvent&){
  for(int i = 0; i < robot_info_.size(); i++){
    robot_info_[i].reload_time = 0;

  }
  ROS_WARN("Resetting reload");
}

void SimNode::gameEnd(const ros::TimerEvent&, int i){
  roborts_sim::Countdown cdm;
  cdm.gameState = "Countdown ends!";
  ROS_INFO("Countdown ends!");
  ros_countdown_pub_[i].publish(cdm);
}

} // roborts_sim

int main(int argc, char **argv) {
  ros::init(argc, argv, "sim_node");
  roborts_sim::SimNode SimNode("sim_node");
  ros::AsyncSpinner async_spinner(THREAD_NUM);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}
