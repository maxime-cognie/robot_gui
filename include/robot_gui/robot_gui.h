#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include "ros/publisher.h"
#include "ros/subscriber.h"
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "geometry_msgs/Twist.h"

class RobotGui {
public:
    RobotGui(ros::NodeHandle *nh, std::string robot_info_topic = "robot_info", std::string pub_vel_topic = "cmd_vel");
    void run();

private:
    ros::NodeHandle *nh_;
    ros::Subscriber robot_info_sub_;
    ros::Publisher cmd_vel_pub_;
    std::string robot_info_topic_;
    std::string cmd_vel_topic_;
    robotinfo_msgs::RobotInfo10Fields robot_info_;
    geometry_msgs::Twist cmd_vel_;
    void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);

    const std::string WINDOW_NAME = "ROBOT GUI";
};
#endif // ROBOT_GUI_H