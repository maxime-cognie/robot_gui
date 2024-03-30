#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#define CVUI_IMPLEMENTATION
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_srvs/Trigger.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class RobotGui {
public:
  RobotGui(ros::NodeHandle *nh,
           const std::string robot_info_topic = "robot_info",
           const std::string pub_vel_topic = "cmd_vel",
           const std::string odom_topic = "odom",
           const std::string dist_track_srv = "get_distance");
  void run();

private:
  ros::NodeHandle *nh_;
  ros::Subscriber robot_info_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::ServiceClient dist_track_client_;

  std::string robot_info_topic_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string dist_track_srv_;

  robotinfo_msgs::RobotInfo10Fields robot_info_;
  geometry_msgs::Twist cmd_vel_;
  nav_msgs::Odometry rob_pos_;
  std_srvs::Trigger trig_srv_;
  
  void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  const std::string WINDOW_NAME = "ROBOT GUI";
};
#endif // ROBOT_GUI_H