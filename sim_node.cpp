#include "sim_node.h"

namespace roborts_sim{

SimNode::SimNode(std::string name) {
  Init();
  ROS_INFO("Simulation node initialized");
  //StartSim();

  std::thread game_thread(&SimNode::GameCountDown, this);
  game_thread.detach();

  std::thread status_thread(&SimNode::ExecutePublishLoop, this);
  status_thread.detach();

  std::thread red_reload_thread(boost::bind(&SimNode::ReloadDetector, this, true));
  red_reload_thread.detach();

  std::thread blue_reload_thread(boost::bind(&SimNode::ReloadDetector, this, false));
  blue_reload_thread.detach();

  std::thread red_buff_thread(boost::bind(&SimNode::BuffzoneDetector, this, true));
  red_buff_thread.detach();
  std::thread blue_buff_thread(boost::bind(&SimNode::BuffzoneDetector, this, true));
  blue_buff_thread.detach();
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
    std::string reload_name               = "/" + robot_info_[i].name + "/" + "reload";
    std::string bonus_topic               = "/" + robot_info_[i].name + "/" + "robot_bonus";
    std::string bonus_status_topic        = '/' + robot_info_[i].name + "/" + "field_bonus_status";
    std::string game_status_topic         = "/" + robot_info_[i].name + "/" + "game_status";
    std::string game_survivor_topic       = "/" + robot_info_[i].name + "/" + "game_survivor";
    std::string supplier_status_topic     = "/" + robot_info_[i].name + "/" + "field_supplier_status";
    // fill up the vectors
    gazebo_real_pose_sub_.push_back(nh_.subscribe<nav_msgs::Odometry>(pose_topic, 100, boost::bind(&SimNode::PoseCallback,this,_1,i)));
    ros_gimbal_angle_sub_.push_back(nh_.subscribe(gimbal_angle_topic, 1, &SimNode::GimbalAngleCtrlCallback, this));
    ros_gimbal_mode_srv_.push_back(nh_.advertiseService(gimbal_mode_service_name, &SimNode::SetGimbalModeService, this));
    ros_ctrl_fric_wheel_srv_.push_back(nh_.advertiseService(fric_wheel_service_name, &SimNode::CtrlFricWheelService, this));
    //ros_ctrl_shoot_srv_.push_back(nh_.advertiseService(gimbal_shoot_service_name, &SimNode::CtrlShootService, this));
    ros_ctrl_shoot_srv_.push_back(nh_.advertiseService<roborts_msgs::ShootCmd::Request,roborts_msgs::ShootCmd::Response>(gimbal_shoot_service_name, boost::bind(&SimNode::CtrlShootService,  this, _1, _2, i+1)));
    
    
    ros_countdown_pub_.push_back(nh_.advertise<roborts_sim::Countdown>(countdown_topic_name, 1000));
    ros_robot_status_pub_.push_back(nh_.advertise<roborts_msgs::RobotStatus>(robot_status_topic_name, 30));
    ros_robot_damage_pub_.push_back(nh_.advertise<roborts_msgs::RobotDamage>(robot_damage_topic_name, 30));
    ros_robot_heat_pub_.push_back(nh_.advertise<roborts_msgs::RobotHeat>(robot_heat_topic_name, 30));
    ros_robot_bonus_pub_.push_back(nh_.advertise<roborts_msgs::RobotBonus>(bonus_topic,30));
    ros_robot_bonus_status_pub_.push_back(nh_.advertise<roborts_msgs::BonusStatus>(bonus_status_topic,30));
    ros_robot_game_status_pub_.push_back(nh_.advertise<roborts_msgs::GameStatus>(game_status_topic, 30));
    ros_robot_game_survivor_pub_.push_back(nh_.advertise<roborts_msgs::GameSurvivor>(game_survivor_topic, 30));
    ros_robot_supplier_status_pub_.push_back(nh_.advertise<roborts_msgs::SupplierStatus>(supplier_status_topic, 30));
    // fill up reload vector
    //reload_srv_.push_back(nh_.advertiseService<roborts_sim::ReloadCmd::Request, roborts_sim::ReloadCmd::Response>(reload_name, boost::bind(&SimNode::ReloadCmd,  this, _1, _2, i+1)));
  }
  // for visualization
  path_pub_ = nh_.advertise<nav_msgs::Path>("los_path", 10);


  // following services are deprecated; we should stick to the protocols used
  // on real robots, rather than define our own
  shoot_srv_ = nh_.advertiseService("shoot", &SimNode::ShootCmd, this);

  //for(int i = 1; i< 5; i++){
    //std::string reload_namespace = "/r" + i + "/reload";
    //reload_srv_.push_back(nh_.advertiseService(reload_namespace, boost::bind(&SimNode::ReloadCmd, _1, _2, i), this));
  //}

  // Advertise Check Bullet service
  check_bullet_srv_ = nh_.advertiseService("check_bullet",&SimNode::CheckBullet,this);
  //ROS_INFO("check bullet service ready");

  // reloadTime.push_back(0);
  // reloadTime.push_back(0);
  // reloadTime.push_back(0);
  // reloadTime.push_back(0);

  //CountDown();

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

void SimNode::ExecutePublishLoop() {
  ros::Rate r(10);
  while(ros::ok()) {
    for(unsigned id = 1; id <= ROBOT_NUM; id++) {
      PublishRobotStatus(id);
      PublishRobotHeat(id);
      SettleRobotHeat(id);
      PublishBonus(id);
      PublishBonusStatus(id);
    }
    ros::spinOnce();
    r.sleep();
  }
}

//void SimNode::StartThread() {
//  for(unsigned i = 0; i < ROBOT_NUM; i++) {
//    robot_medium_thread_.push_back(std::thread(&SimNode::ExecuteLoop, this, i));
//  }
//}
//
//void SimNode::StopThread() {
//  for(unsigned i = 0; i < ROBOT_NUM; i++) {
//    if (robot_medium_thread_[i].joinable()) {
//      robot_medium_thread_[i].join();
//    }
//  }
//}
//
//void SimNode::ExecuteLoop(int robot) {
//  ros::Rate r(10);
//  while (ros::ok()) {
//    ROS_INFO("Robot %d %s", robot, __FUNCTION__);
//    PublishRobotStatus(robot);
//    PublishRobotHeat(robot);
//    SettleRobotHeat(robot);
//
//    ros::spinOnce();
//    r.sleep();
//  }
//}

void SimNode::PublishRobotStatus(int robot) {
  roborts_msgs::RobotStatus robot_status;
  // transform literal robot name to numeric id
  if (robot == 1) {
    robot_status.id = 1;
    robot_status.remain_hp = robot_info_[0].hp;
    ros_robot_status_pub_[0].publish(robot_status);
  } else if (robot == 2) {
    robot_status.id = 2;
    robot_status.remain_hp = robot_info_[1].hp;
    ros_robot_status_pub_[1].publish(robot_status);
  } else if (robot == 3) {
    robot_status.id = 13;
    robot_status.remain_hp = robot_info_[2].hp;
    ros_robot_status_pub_[2].publish(robot_status);
  } else if (robot == 4) {
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

void SimNode::PublishGameSurvivor(){
  roborts_msgs::GameSurvivor gs;
  (robot_info_[0].hp == 0) ? gs.red3 = false : gs.red3 = true;
  (robot_info_[1].hp == 0) ? gs.red4 = false : gs.red4 = true;
  (robot_info_[2].hp == 0) ? gs.blue3 = false : gs.blue3 = true;
  (robot_info_[3].hp == 0) ? gs.blue4 = false : gs.blue4 = true;
  for(int i = 0; i < ROBOT_NUM; i++){
    ros_robot_game_survivor_pub_[i].publish(gs);
  }
}

void SimNode::PublishGameStatus(int robot){
  roborts_msgs::GameStatus gsmsg;
  if(remaining_time > 300){
    gsmsg.game_status = 3;
    gsmsg.remaining_time = remaining_time;
    if(robot == 1){
      ROS_INFO("Game starts in %d second(s).", remaining_time - 300);
    }
  }else{
    gsmsg.game_status = 4;
    gsmsg.remaining_time = remaining_time;
  }
  ros_robot_game_status_pub_[robot].publish(gsmsg);
}

void SimNode::PublishBonus(int robot){
  roborts_msgs::RobotBonus rb;
  rb.bonus = robot_info_[robot-1].bonus;
  ros_robot_bonus_pub_[robot-1].publish(rb);
}

void SimNode::PublishBonusStatus(int robot){
  ros_robot_bonus_status_pub_[robot-1].publish(bs);
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
  int barrel_heat;
  {
    std::lock_guard <std::mutex> guard(mutex_);
    barrel_heat = robot_info_[robot - 1].barrel_heat;
    int robot_hp = robot_info_[robot - 1].hp;
    if (barrel_heat > 720) {
      ROS_WARN("There must be a synchronization problem.");
    }
    if (robot_hp >= 400) {
      robot_info_[robot - 1].barrel_heat = std::max(robot_info_[robot - 1].barrel_heat - static_cast<int>(BARREL_COOLING_RATE / 2), 0);
    } else {
      robot_info_[robot - 1].barrel_heat = std::max(robot_info_[robot - 1].barrel_heat - BARREL_COOLING_RATE, 0);
    }
  }
  if (barrel_heat > 360) {
    HpDown(robot, ComputeBarrelDamage(barrel_heat), 2);
  }
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

void SimNode::HpDown(int robot, int damage, int damage_type, int damage_source) {
  if (robot_info_[robot-1].hp <= 0) {
    ROS_WARN("Actually Robot %d has been dead but still received damage", robot);
  }

  {
    std::lock_guard <std::mutex> guard(mutex_);
    robot_info_[robot-1].hp -= damage;
  }
  roborts_msgs::RobotDamage robot_damage;
  robot_damage.damage_type = damage_type;
  robot_damage.damage_source = damage_source;
  ros_robot_damage_pub_[robot-1].publish(robot_damage);
  if (robot_info_[robot-1].hp < 0) {
    robot_info_[robot-1].hp = 0;
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
    roborts_msgs::SupplierStatus ss1;
    ss1.status = 2;
    ros_robot_supplier_status_pub_[robot-1].publish(ss1);
    std::chrono::milliseconds dura(5000);
    std::this_thread::sleep_for(dura);
    robot_info_[robot-1].ammo = 50;
    roborts_msgs::SupplierStatus ss2;
    ss2.status = 0;
    ros_robot_supplier_status_pub_[robot-1].publish(ss2);
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
                              roborts_msgs::ShootCmd::Response &res,
                              const int robot_index){
  if (robot_info_[robot_index-1].ammo <= 0) {
    ROS_WARN("Robot %d has no ammo but tries to shoot.", robot_index);
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> currentPath;
  // the offset between robot armor and its base link
  double fb_offset = 0.25;
  double lr_offset = 0.15;
  double armor_width = 0.07;
  RobotInfo robot = robot_info_[robot_index-1];
                             
  for(auto target:robot_info_)
  {
    if (target.name!=robot.name)
    {
      auto r1_pos = robot.pose.pose.position;
      auto r2_pos = target.pose.pose.position;
      if (map_.hasLineOfSight(r1_pos.x, r1_pos.y, r2_pos.x, r2_pos.y, currentPath))
      {
        std::string id_str = target.name;
        id_str.erase(0,1);
        int robot_id = std::stoi(id_str);

        double r1_yaw = GetYaw(robot.pose.pose.orientation);
        double r2_yaw = GetYaw(target.pose.pose.orientation);
        double pi = PI;

        r2_yaw = r2_yaw + PI;
        if (r2_yaw > PI) r2_yaw=r2_yaw-2*PI;

        //front 
        double r2_f_x = r2_pos.x+cos(r2_yaw)*fb_offset;
        double r2_f_y = r2_pos.y+sin(r2_yaw)*fb_offset;
        double dis_f = sqrt(pow(r2_f_x - r1_pos.x,2)+pow(r2_f_y - r1_pos.y,2));

        //left
        double r2_yaw_l = r2_yaw + 90/180.0*pi;

        if(r2_yaw_l>PI) r2_yaw_l = r2_yaw_l - 2 * PI;
        double r2_l_x = r2_pos.x+cos(r2_yaw_l)*lr_offset;
        double r2_l_y = r2_pos.y+sin(r2_yaw_l)*lr_offset;
        double dis_l = sqrt(pow(r2_l_x - r1_pos.x,2)+pow(r2_l_y - r1_pos.y,2));
        
        //back
        double r2_yaw_b = r2_yaw_l + 90/180.0*pi;
        if(r2_yaw_b>PI) r2_yaw_b = r2_yaw_b - 2 * PI;
        double r2_b_x = r2_pos.x+cos(r2_yaw_b)*fb_offset;
        double r2_b_y = r2_pos.y+sin(r2_yaw_b)*fb_offset;
        double dis_b = sqrt(pow(r2_b_x - r1_pos.x,2)+pow(r2_b_y - r1_pos.y,2));

        //right 
        double r2_yaw_r = r2_yaw_b + 90/180.0*pi;
        if(r2_yaw_r>PI) r2_yaw_r = r2_yaw_r - 2 * PI;        
        double r2_r_x = r2_pos.x+cos(r2_yaw_r)*lr_offset;
        double r2_r_y = r2_pos.y+sin(r2_yaw_r)*lr_offset;
        double dis_r = sqrt(pow(r2_r_x - r1_pos.x,2)+pow(r2_r_y - r1_pos.y,2));
        ROS_WARN("The ywa: f: %f, l: %f, b: %f, r: %f",r2_yaw,r2_yaw_l,r2_yaw_b,r2_yaw_r);


        double i_x,i_y;
        if (dis_f < dis_l && dis_f < dis_b && dis_f < dis_r)
        {
          i_x = (tan(r1_yaw)*r1_pos.x-tan(r2_yaw_l)*r2_f_x+r2_f_y-r1_pos.y)/(tan(r1_yaw)-tan(r2_yaw_l));
          i_y = tan(r1_yaw)*(i_x-r1_pos.x)+r1_pos.y;
          if (sqrt(pow(r2_f_x - i_x,2)+pow(r2_f_y - i_y,2))<armor_width)
          {
            ROS_INFO("Valid hit, damage detected on %s front armor",target.name.c_str());
            ROS_WARN("current yaw for target is %.5f",r2_yaw);
            AmmoDown(robot_index, 1);
            HpDown(robot_id, DAMAGE_PER_BULLET, 0, 0);
          }
          else
          {
            ROS_INFO("%s trying to shoot front armor, but missed!", robot.name.c_str());
            ROS_WARN("current yaw for target is %.5f",r2_yaw);
          }
          
        }
        else if (dis_l < dis_f &&dis_l < dis_b &&dis_l < dis_r)
        {
          i_x = (tan(r1_yaw)*r1_pos.x-tan(r2_yaw)*r2_l_x+r2_l_y-r1_pos.y)/(tan(r1_yaw)-tan(r2_yaw));
          i_y = tan(r1_yaw)*(i_x-r1_pos.x)+r1_pos.y;
          if (sqrt(pow(r2_l_x - i_x,2)+pow(r2_l_y - i_y,2))<armor_width)
          {
            ROS_INFO("Valid hit, damage detected on %s left armor",target.name.c_str());
            ROS_WARN("current yaw for target is %.5f",r2_yaw_l);
            AmmoDown(robot_index, 1);
            HpDown(robot_id, DAMAGE_PER_BULLET, 0, 1);
//            roborts_msgs::RobotDamage dmg_msg;
//            dmg_msg.damage_type = 0;
//            dmg_msg.damage_source = 1;
//            ros_robot_damage_pub_[robot_id-1].publish(dmg_msg);
          }
          else
          {
            ROS_INFO("%s trying to shoot left armor, but missed! dis is: %f", robot.name.c_str(),sqrt(pow(r2_r_x - i_x,2)+pow(r2_r_y - i_y,2)));
            ROS_WARN("current yaw for target is %.5f",r2_yaw_l);
          }
        }
        else if (dis_b < dis_l &&dis_b < dis_f &&dis_b < dis_r)
        {
          i_x = (tan(r1_yaw)*r1_pos.x-tan(r2_yaw_l)*r2_b_x+r2_b_y-r1_pos.y)/(tan(r1_yaw)-tan(r2_yaw_l));
          i_y = tan(r1_yaw)*(i_x-r1_pos.x)+r1_pos.y;
          if (sqrt(pow(r2_b_x - i_x,2)+pow(r2_b_y - i_y,2))<armor_width)
          {
            ROS_INFO("Valid hit, damage detected on %s back armor",target.name.c_str());
            ROS_WARN("current yaw for target is %.5f",r2_yaw_b);
            AmmoDown(robot_index, 1);
            HpDown(robot_id, DAMAGE_PER_BULLET, 0, 2);
//            roborts_msgs::RobotDamage dmg_msg;
//            dmg_msg.damage_type = 0;
//            dmg_msg.damage_source = 2;
//            ros_robot_damage_pub_[robot_id-1].publish(dmg_msg);
          }
          else
          {
            ROS_INFO("%s trying to shoot back armor, but missed!", robot.name.c_str());
            ROS_WARN("current yaw for target is %.5f",r2_yaw_b);
          }
        }
        else if (dis_r < dis_l &&dis_r < dis_b &&dis_r < dis_f)
        {
          i_x = (tan(r1_yaw)*r1_pos.x-tan(r2_yaw)*r2_r_x+r2_r_y-r1_pos.y)/(tan(r1_yaw)-tan(r2_yaw));
          i_y = tan(r1_yaw)*(i_x-r1_pos.x)+r1_pos.y;
          if (sqrt(pow(r2_r_x - i_x,2)+pow(r2_r_y - i_y,2))<armor_width)
          {
            ROS_INFO("Valid hit, damage detected on %s right armor",target.name.c_str());
            ROS_WARN("current yaw for target is %.5f",r2_yaw_r);
            AmmoDown(robot_index, 1);
            HpDown(robot_id, DAMAGE_PER_BULLET, 0, 3);
//            roborts_msgs::RobotDamage dmg_msg;
//            dmg_msg.damage_type = 0;
//            dmg_msg.damage_source = 3;
//            ros_robot_damage_pub_[robot_id-1].publish(dmg_msg);
          }
          else
          {
            ROS_INFO("%s trying to shoot right armor, but missed!", robot.name.c_str());
            ROS_WARN("current yaw for target is %.5f",r2_yaw_r);
          }
        }
        else
        {
          ROS_WARN("The distances: f: %f, l: %f, b: %f, r: %f",r2_yaw,r2_yaw_l,r2_yaw_b,r2_yaw_r);
        }
        
        // check the side armor
      }
    }
  }
  
  //ROS_WARN("Robot :%d trying to shoot",robot);
  return true;
}

double SimNode::GetYaw(geometry_msgs::Quaternion orientation)
{
  tf::Quaternion q(orientation.x,orientation.y,orientation.z,orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
}

bool SimNode::ShootCmd(roborts_msgs::ShootCmdSim::Request &req,
                  roborts_msgs::ShootCmdSim::Response &res){
  int robot1 = req.robot;
  int robot2 = req.enemy;
  res.success = TryShoot(robot1, robot2);
  return true;
}
/*
bool SimNode::ReloadCmd(roborts_sim::ReloadCmd::Request &req, 
                  roborts_sim::ReloadCmd::Response &res,
                  int robot){
  //int robot = req.robot;
  res.success = TryReload(robot);
  res.supply_projectile_id = 1;
  res.supply_robot_id = req.robot;
  res.supply_num = 50;
  return true;
}
*/
bool SimNode::ReloadDetector(bool red){
  ros::Rate r(50);
  geometry_msgs::PoseStamped reload_spot;
  int robot1;
  int robot2;
  if(red == true){
    reload_spot.pose.position.x = 5;
    reload_spot.pose.position.y = 5;
    reload_spot.pose.position.z = 0;
    robot1 = 0;
    robot2 = 1;
  }else{
    reload_spot.pose.position.x = 5;
    reload_spot.pose.position.y = 0;
    reload_spot.pose.position.z = 0;
    robot1 = 2;
    robot2 = 3;
  }
  while(ros::ok()){
    if(pow(robot_info_[robot1].pose.pose.position.x - reload_spot.pose.position.x, 2) +
        pow(robot_info_[robot1].pose.pose.position.y - reload_spot.pose.position.y, 2) < 0.17){
      if(robot_info_[robot1].reload_time>1){
        ROS_INFO("Robot %d has reloaded for 2 times in 1 minute, failed.", robot1);
        roborts_msgs::SupplierStatus ss;
        ss.status = 0;
        ros_robot_supplier_status_pub_[robot1].publish(ss);
      }
      else{
        TryReload(robot1);
      }
    }
    if(pow(robot_info_[robot2].pose.pose.position.x - reload_spot.pose.position.x, 2) +
        pow(robot_info_[robot2].pose.pose.position.y - reload_spot.pose.position.y, 2) < 0.17){
      if(robot_info_[robot2].reload_time>1){
        ROS_INFO("Robot %d has reloaded for 2 times in 1 minute, failed.", robot2);
        roborts_msgs::SupplierStatus ss;
        ss.status = 0;
        ros_robot_supplier_status_pub_[robot2].publish(ss);
      }
      else{
        TryReload(robot2);
      }
    }
    r.sleep();
  }
}

bool SimNode::BuffzoneDetector(bool red){
  ros::Rate r(10);
  geometry_msgs::PoseStamped buff_zone;
  if(red == true){
    buff_zone.pose.position.x = 6.3;
    buff_zone.pose.position.y = 1.125;
    buff_zone.pose.position.z = 0;
  }else{
    buff_zone.pose.position.x = 1.7;
    buff_zone.pose.position.y = 3.875;
    buff_zone.pose.position.z = 0;
  }
  while(ros::ok()){
    if(red){
      bs.red_bonus = 0;
    }else{
      bs.blue_bonus = 0;
    }
    for(int i=0; i<4; i++){
      if(pow(robot_info_[i].pose.pose.position.x - buff_zone.pose.position.x, 2) +
        pow(robot_info_[i].pose.pose.position.y - buff_zone.pose.position.y, 2) < 0.17){
        if(red){
            bs.red_bonus = 2;
          }else{
            bs.blue_bonus = 2;
          }
        if(robot_info_[0].buff_time>0){
          ROS_INFO("Red team has got bonus in 1 minute, failed.");
        }
        else{
          if(red){
            TryRedBuff();
          }
          else{
            TryBlueBuff();
          }
        }
      }
    }
    r.sleep();
  }
}

void SimNode::TryRedBuff(){
  ROS_INFO("Red team tries buffing");
  robot_info_[0].buff_time++;
  robot_info_[1].buff_time++;
  std::chrono::milliseconds dura(5000);
  std::this_thread::sleep_for(dura);
  robot_info_[0].bonus = true;
  robot_info_[1].bonus = true;
  red_bonus_time = remaining_time;

    
}

void SimNode::TryBlueBuff(){
  ROS_INFO("Blue team tries buffing");
  robot_info_[2].buff_time++;
  robot_info_[3].buff_time++;
  std::chrono::milliseconds dura(5000);
  std::this_thread::sleep_for(dura);
  robot_info_[2].bonus = true;
  robot_info_[3].bonus = true;
  blue_bonus_time = remaining_time;
}

void SimNode::resetBuff(bool red){
  if(red){
    robot_info_[0].bonus = false;
    robot_info_[1].bonus = false;
  }
  else{
    robot_info_[2].bonus = false;
    robot_info_[3].bonus = false;
  }
  ROS_INFO("Bonus reseted");
}

void SimNode::resetBufftime(){
  for(int i = 0; i < robot_info_.size(); i++){
    robot_info_[i].buff_time = 0;
  }
  ROS_INFO("Bonus times reseted");
}
/*void SimNode::CountDown(){
  roborts_sim::Countdown cdm;
  cdm.gameState = "Game starts!";
  ROS_INFO("Game starts!");


  // for (auto & pub : ros_countdown_pub_) {
  //   pub.publish(cdm);
  // }
  
  //ros::spinOnce();
  //ros::spin();
  //nh_.createTimer(ros::Duration(120), &SimNode::resetReload, this);
  //ros::spinOnce();
  

    //cd.publish(cdm);
  
  for (int i = 0; i < ros_countdown_pub_.size(); i ++) {
    ros_countdown_pub_[i].publish(cdm);
    ROS_INFO("%s", cdm.gameState.c_str());
    countdown_timer_.push_back(nh_.createTimer(ros::Duration(180), boost::bind(&SimNode::gameEnd, this, _1, i)));
    ros::spinOnce();
    //ros::spin();
  }
  reload_timer_ = nh_.createTimer(ros::Duration(60), &SimNode::resetReload, this);
  ros::spinOnce();
}*/

void SimNode::resetReload(){
  for(int i = 0; i < robot_info_.size(); i++){
    robot_info_[i].reload_time = 0;
  }
  ROS_INFO("Reload times reseted");
}

/*void SimNode::gameEnd(const ros::TimerEvent&, int i){
  roborts_sim::Countdown cdm;
  cdm.gameState = "Countdown ends!";
  ROS_INFO("Countdown ends!");
  ros_countdown_pub_[i].publish(cdm);
}*/

void SimNode::GameCountDown(){
  ros::Rate r(1);
  while(ros::ok()){
    if(remaining_time % 60 == 0){
      resetReload();
      resetBufftime();
    }
    if(remaining_time == red_bonus_time - 60){
      resetBuff(true);
    }
    if(remaining_time == blue_bonus_time - 60){
      resetBuff(false);
    }
    PublishGameSurvivor();
    for(int i = 1; i <= ROBOT_NUM; i++){
      PublishGameStatus(i);
    }
    remaining_time--;
    if(remaining_time == 0){
      for(int i = 1; i <= ROBOT_NUM; i++){
        PublishGameStatus(i);
      }
      return;
    }
    r.sleep();
  }

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
