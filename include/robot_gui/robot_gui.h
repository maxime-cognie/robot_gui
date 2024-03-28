#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "robotinfo_msgs/RobotInfo10Fields.h"

class RobotGui {
public:
    RobotGui(ros::NodeHandle *nh, std::string robot_info_topic = "robot_info");
    void run();

private:
    ros::NodeHandle *nh_;
    ros::Subscriber robot_info_sub_;
    std::string robot_info_topic_ = "";
    robotinfo_msgs::RobotInfo10Fields robot_info_;
    void robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
    const std::string WINDOW_NAME = "ROBOT GUI";
};
#endif // ROBOT_GUI_H