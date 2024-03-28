#include "robot_gui/robot_gui.h"

RobotGui::RobotGui(ros::NodeHandle *nh, std::string robot_info_topic) {
  this->nh_ = nh;
  this->robot_info_topic_ = robot_info_topic;
  this->robot_info_sub_ = nh_->subscribe(this->robot_info_topic_, 10,
                                         &RobotGui::robotInfoCallback, this);
}

void RobotGui::robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  this->robot_info_ = *msg;
}

void RobotGui::run() {
  cv::Mat frame = cv::Mat(140, 475, CV_8UC3);
  int scale = 15;

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Clear the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Create window at (20, 20) with size 435x100 (width x height) and title
    cvui::window(frame, 20, 20, 435, 100, "General Info :");
    
    cvui::printf(frame, 25, 45, 0.4, 0xffffff, this->robot_info_.data_field_01.c_str());
    cvui::printf(frame, 25, 45 + 1 * scale, 0.4, 0xffffff, this->robot_info_.data_field_02.c_str());
    cvui::printf(frame, 25, 45 + 2 * scale, 0.4, 0xffffff, this->robot_info_.data_field_03.c_str());
    cvui::printf(frame, 25, 45 + 3 * scale, 0.4, 0xffffff, this->robot_info_.data_field_04.c_str());
    cvui::printf(frame, 25, 45 + 4 * scale, 0.4, 0xffffff, this->robot_info_.data_field_05.c_str());
    cvui::printf(frame, 240, 45, 0.4, 0xffffff, this->robot_info_.data_field_06.c_str());
    cvui::printf(frame, 240, 45 + 1 * scale, 0.4, 0xffffff, this->robot_info_.data_field_07.c_str());
    cvui::printf(frame, 240, 45 + 2 * scale, 0.4, 0xffffff, this->robot_info_.data_field_08.c_str());
    cvui::printf(frame, 240, 45 + 3 * scale, 0.4, 0xffffff, this->robot_info_.data_field_09.c_str());
    cvui::printf(frame, 240, 45 + 4 * scale, 0.4, 0xffffff, this->robot_info_.data_field_10.c_str());

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
  }
}