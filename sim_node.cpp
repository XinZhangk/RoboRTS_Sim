#include "sim_node.h"

namespace roborts_sim{

SimNode::SimNode(std::string name) {
  Init();
  ROS_INFO("initialize simulation node");
  //StartSim();

}

bool SimNode::Init() {
  // request the static map
  GetStaticMap();
  // initialize robot informatoin
  int hp, ammo_count;
  std::vector<std::string> name_list;
  std::vector<std::string> color_list;
  if(nh_.getParam("/hp", hp) &&
     nh_.getParam("/ammo", ammo_count) &&
     nh_.getParam("/name_list", name_list) &&
     nh_.getParam("/color_list", color_list)){
    ROS_INFO("robot parameters loaded");
  }else{
    ROS_ERROR("can not read robot parameters");
  }
  // initialize robot information
  for(unsigned i = 0; i < name_list.size(); i++){
    Color color = StrToColor(color_list[i]);
    robot_info_.push_back(RobotInfo(name_list[i], color, ammo_count, hp));
  }

  // create subscribers and services
  for (unsigned i = 0; i < robot_info_.size(); i++) {
    // initialize subscribers and service severs
    std::string pose_topic                = "/" + robot_info_[i].name + "/" + "gazebo_robot_pose";
    std::string gimbal_angle_topic        = "/" + robot_info_[i].name + "/" + "cmd_gimbal_angle";
    std::string gimbal_mode_service_name  = "/" + robot_info_[i].name + "/" + "set_gimbal_mode";
    std::string fric_wheel_service_name   = "/" + robot_info_[i].name + "/" + "cmd_fric_wheel";
    std::string gimbal_shoot_service_name = "/" + robot_info_[i].name + "/" + "cmd_shoot";
    // fill up the vectors
    gazebo_real_pose_sub_.push_back(nh_.subscribe<nav_msgs::Odometry>(pose_topic, 100, boost::bind(&SimNode::PoseCallback,this,_1,i)));
    ros_gimbal_angle_sub_.push_back(nh_.subscribe(gimbal_angle_topic, 1, &SimNode::GimbalAngleCtrlCallback, this));
    ros_gimbal_mode_srv_.push_back(nh_.advertiseService(gimbal_mode_service_name, &SimNode::SetGimbalModeService, this));
    ros_ctrl_fric_wheel_srv_.push_back(nh_.advertiseService(fric_wheel_service_name, &SimNode::CtrlFricWheelService, this));
    ros_ctrl_shoot_srv_.push_back(nh_.advertiseService(gimbal_shoot_service_name, &SimNode::CtrlShootService, this));
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

  std::thread(CountDown());
  return true;
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

// void SimNode::StartSim() {

// }

void SimNode::AmmoDown(int robot, int num) {
  robot_info_[robot-1].ammo -= num;
}
void SimNode::HpDown(int robot, int damage){
  robot_info_[robot-1].hp -= damage;
  ROS_INFO("Robot %d lost %d hit points", robot, damage);
}

bool SimNode::TryShoot(int robot1, int robot2) {
  ROS_INFO("Robot 1 tries to shoot Robot 2");
  std::vector<geometry_msgs::PoseStamped> currentPath;
  auto r1_pos = robot_info_[robot1-1].pose.pose.position;
  auto r2_pos = robot_info_[robot2-1].pose.pose.position;
  if (map_.hasLineOfSight(r1_pos.x, r1_pos.y, r2_pos.x, r2_pos.y, currentPath)){
    ROS_INFO("Robot %d and Robot %d can see each other", robot1, robot2);
    HpDown(robot2, 1);
    // comment out bullet down for now. 
    // ToDo: add reload zone and reload functionality for the sim node
    //AmmoDown(robot1, 1);
  } else {
    ROS_INFO("Robot %d and Robot %d cannot see each other", robot1, robot2);
  }
  PublishPath(currentPath);
}

bool SimNode::TryReload(int robot){
  ROS_INFO("Robot %d tries reloading.", robot);
  if(reloadTime[robot-1] > 1){
    ROS_INFO("Robot %d has reloaded for 2 times in 1 minute, failed.", robot);
    return false;
  }else{
    reloadTime[robot-1]++;
    ROS_INFO("Robot %d has reloaded for %d times in 1 minute, succeed.", robot, reloadTime[robot-1]);
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
  return true;
}

void SimNode::CountDown(){
  ros::Publisher cd = nh_.advertise<roborts_sim::Countdown>("countdown", 1000);
  roborts_sim::Countdown cdm;
  cdm.gameState = "Countdown starts!";
  ROS_INFO("Countdown starts!");
  cd.publish(cdm);
  time_t t;
    while(m>=0){
      t = time(NULL);//get current time
        while(time(NULL)==t);
        if(--s<0){
            s=59;
            m--;
            for(int i = 0; i < 4; i++){
              reloadTime[i] = 0;
            }
        }
    } 
  cdm.gameState = "Countdown ends!";
  ROS_INFO("Countdown ends!");
  cd.publish(cdm);
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
