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
  // initilize robot informatoin
  robot_info_.push_back(RobotInfo("r1", red, 100, 50));
  robot_info_.push_back(RobotInfo("r2", red, 100, 50));
  robot_info_.push_back(RobotInfo("r3", blue, 100, 50));
  robot_info_.push_back(RobotInfo("r4", blue, 100, 50));

  sub_r1_ = nh_.subscribe<nav_msgs::Odometry>("/r1/gazebo_robot_pose", 100, boost::bind(&SimNode::PoseCallback,this,_1,1));
  sub_r2_ = nh_.subscribe<nav_msgs::Odometry>("/r2/gazebo_robot_pose", 100, boost::bind(&SimNode::PoseCallback,this,_1,2));
  sub_r3_ = nh_.subscribe<nav_msgs::Odometry>("/r3/gazebo_robot_pose", 100, boost::bind(&SimNode::PoseCallback,this,_1,3));
  sub_r4_ = nh_.subscribe<nav_msgs::Odometry>("/r4/gazebo_robot_pose", 100, boost::bind(&SimNode::PoseCallback,this,_1,4));

  shoot_srv_ = nh_.advertiseService("shoot", &SimNode::ShootCmd, this);

  path_pub_ = nh_.advertise<nav_msgs::Path>("los_path", 10);
 
  return true;
}

bool SimNode::GetStaticMap(){
  static_map_srv_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
  ros::service::waitForService("/static_map", -1);
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;
  if(static_map_srv_.call(req,res)) {
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

void SimNode::StartSim() {

}

void SimNode::BulletDown(int robot, int num) {
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
    HpDown(2, 1);
  } else {
    ROS_INFO("Robot %d and Robot %d cannot see each other", robot1, robot2);
  }
  PublishPath(currentPath);
}

// Use a universal pose callback method
void SimNode::PoseCallback(const nav_msgs::Odometry::ConstPtr &pose_msg,const int topic){
  auto position = pose_msg->pose.pose.position;
  auto orientation = pose_msg->pose.pose.orientation;
  robot_info_[topic-1].pose = geometry_msgs::PoseWithCovariance(pose_msg->pose);
  auto pose = robot_info_[topic-1].pose;
  //ROS_INFO("r[%d] has pose [%.4f ,%.4f ,%.4f]", topic,pose.pose.position.x, pose.pose.position.y, pose.pose.position.z );
}

bool SimNode::ShootCmd(roborts_msgs::ShootCmdSim::Request &req,
                  roborts_msgs::ShootCmdSim::Response &res){
  int robot1 = req.robot;
  int robot2 = req.enemy;
  res.success = TryShoot(robot1, robot2);
  return true;
}

} // roborts_sim

int main(int argc, char **argv) {
  //roborts_localization::GLogWrapper glog_wrapper(argv[0]);
  ros::init(argc, argv, "sim_node");
  roborts_sim::SimNode SimNode("sim_node");
  ros::AsyncSpinner async_spinner(THREAD_NUM);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}
