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
  robot_info_.push_back(RobotInfo("r1", red, 100, 100));
  robot_info_.push_back(RobotInfo("r2", red, 100, 100));
  robot_info_.push_back(RobotInfo("r3", blue, 100, 100));
  robot_info_.push_back(RobotInfo("r4", blue, 100, 100));

  sub_r1_ = nh_.subscribe("/r1/gazebo_robot_pose", 100, &SimNode::PoseCallback1, this);
  sub_r2_ = nh_.subscribe("/r2/gazebo_robot_pose", 100, &SimNode::PoseCallback2, this);
  sub_r3_ = nh_.subscribe("/r3/gazebo_robot_pose", 100, &SimNode::PoseCallback3, this);
  sub_r4_ = nh_.subscribe("/r4/gazebo_robot_pose", 100, &SimNode::PoseCallback4, this);

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

void SimNode::CheckLOS(int robot1, int robot2) {
  std::vector<geometry_msgs::PoseStamped> currentPath;
  auto r1_pos = robot_info_[robot1-1].pose.pose.position;
  auto r2_pos = robot_info_[robot2-1].pose.pose.position;
  if (map_.hasLineOfSight(r1_pos.x, r1_pos.y, r2_pos.x, r2_pos.y, currentPath)){
    ROS_INFO("r%d and r%d can see each other", robot1, robot2);
  } else {
    ROS_INFO("r%d and r%d cannot see each other", robot1, robot2);
  }
  PublishPath(currentPath);
}


// todo: make the pose callback better and less repetitive
void SimNode::PoseCallback1(const nav_msgs::Odometry::ConstPtr &pose_msg){
  auto position = pose_msg->pose.pose.position;
  auto orientation = pose_msg->pose.pose.orientation;
  robot_info_[0].pose = geometry_msgs::PoseWithCovariance(pose_msg->pose);
  auto pose = robot_info_[0].pose;
  //ROS_INFO("r1 has pose [%.4f ,%.4f ,%.4f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z );
}
void SimNode::PoseCallback2(const nav_msgs::Odometry::ConstPtr &pose_msg){
  auto position = pose_msg->pose.pose.position;
  auto orientation = pose_msg->pose.pose.orientation;
  robot_info_[1].pose = geometry_msgs::PoseWithCovariance(pose_msg->pose);
  //ROS_INFO("r2 has pose [%.4f ,%.4f ,%.4f]", position.x, position.y, position.z );
}
void SimNode::PoseCallback3(const nav_msgs::Odometry::ConstPtr &pose_msg){
  auto position = pose_msg->pose.pose.position;
  auto orientation = pose_msg->pose.pose.orientation;
  robot_info_[2].pose = geometry_msgs::PoseWithCovariance(pose_msg->pose);
  //ROS_INFO("r3 has pose [%.4f ,%.4f ,%.4f]", position.x, position.y, position.z );
}
void SimNode::PoseCallback4(const nav_msgs::Odometry::ConstPtr &pose_msg){
  auto position = pose_msg->pose.pose.position;
  auto orientation = pose_msg->pose.pose.orientation;
  robot_info_[3].pose = geometry_msgs::PoseWithCovariance(pose_msg->pose);
  //ROS_INFO("r4 has pose [%.4f ,%.4f ,%.4f]", position.x, position.y, position.z );
}

} // roborts_sim

int main(int argc, char **argv) {
  //roborts_localization::GLogWrapper glog_wrapper(argv[0]);
  ros::init(argc, argv, "sim_node");
  roborts_sim::SimNode SimNode("sim_node");
  ros::AsyncSpinner async_spinner(THREAD_NUM);
  async_spinner.start();
  // char command = '0';
  // while (command != 27) {
  //   std::cout << "**************************************************************************************" << std::endl;
  //   std::cout << "*********************************please send a command********************************" << std::endl;
  //   std::cout << "1: test r1 and r2" << std::endl
  //             << "esc: exit program" << std::endl;
  //   std::cout << "**************************************************************************************" << std::endl;
  //   std::cout << "> ";
  //   std::cin >> command;
  //   if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != 27) {
  //     std::cout << "please input again!" << std::endl;
  //     std::cout << "> ";
  //     std::cin >> command;
  //   }
  // }
  ros::waitForShutdown();
  return 0;
}