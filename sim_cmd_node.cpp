#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>


#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "roborts_msgs/ShootCmdSim.h"
#include <ros/ros.h>


int main(int argc, char **argv) {
  ros::init(argc, argv, "sim_cmd_node");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<roborts_msgs::ShootCmdSim>("shoot");

  ros::Rate r(10);
  ROS_INFO("Robot 1 tries to shoot Robot 2");
  while (ros::ok()) {
    // construct srv
    roborts_msgs::ShootCmdSim srv;
    // robots are indexed as 1, 2, 3, 4
    srv.request.robot = 1;
    srv.request.enemy = 2;
    if (client.call(srv)) {
      ROS_INFO("Shooting is successful");
    } else {
      ROS_ERROR("Failed to call service shoot");
      return 1;
    }
    r.sleep();
  }
  return 0;
}