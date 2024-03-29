#include "robot_gui/robot_gui.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

RobotGui::RobotGui(ros::NodeHandle *nh, std::string robot_info_topic,
                   std::string cmd_vel_topic) {
  this->nh_ = nh;
  this->robot_info_topic_ = robot_info_topic;
  this->cmd_vel_topic_ = cmd_vel_topic;
  this->robot_info_sub_ = this->nh_->subscribe(
      this->robot_info_topic_, 10, &RobotGui::robotInfoCallback, this);
  //this->cmv_vel_sub_ = this->nh_->subscribe(this->cmd_vel_topic_, 10, )
  this->cmd_vel_pub_ =
      this->nh_->advertise<geometry_msgs::Twist>(this->cmd_vel_topic_, 1);
  this->cmd_vel_.linear.x = 0;
  this->cmd_vel_.angular.z = 0;
  this->cmd_vel_pub_.publish(this->cmd_vel_);
}

void RobotGui::robotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  this->robot_info_ = *msg;
}

void RobotGui::run() {

  // size of the main window
  int mw_width = 800;
  int mw_height = 1000;

  cv::Mat frame = cv::Mat(mw_height, mw_width, CV_8UC3);

  // line separation
  int step = 5;
  // position and size of the general info window
  int x_win_info = (mw_width - 435) / 2; // center the window
  int y_win_info = 20;
  int win_info_width = 435;
  int win_info_height = 100;

  // teleoparation buttons info
  int x_win_button = 100;
  int y_win_button = y_win_info + win_info_height + 40;
  int button_win_width = mw_width - 2 * x_win_button;
  int button_win_height = 280;
  int inter_button_space = 20;
  int button_width = std::lround((button_win_width - 2 * inter_button_space) / 3);
  int button_height = 80;

      // Init a OpenCV window and tell cvui to use it.
      cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Clear the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    /*  ----------  General info window  ----------  */
    // Create window at (x, y) with size 435x100 (width x height) and title
    cvui::window(frame, x_win_info, y_win_info, win_info_width,
                 win_info_height, "General Info :");

    cvui::beginRow(frame, x_win_info + 5, y_win_info + 25, win_info_width,
                   win_info_height);
    cvui::beginColumn(win_info_width / 2, win_info_height);
    cvui::printf(0.4, 0xffffff, this->robot_info_.data_field_01.c_str());
    cvui::space(step);
    cvui::printf(0.4, 0xffffff, this->robot_info_.data_field_02.c_str());
    cvui::space(step);
    cvui::printf(0.4, 0xffffff, this->robot_info_.data_field_03.c_str());
    cvui::space(step);
    cvui::printf(0.4, 0xffffff, this->robot_info_.data_field_04.c_str());
    cvui::space(step);
    cvui::printf(0.4, 0xffffff, this->robot_info_.data_field_05.c_str());
    cvui::endColumn();
    cvui::beginColumn(win_info_width / 2, win_info_height);
    cvui::printf(0.4, 0xffffff, this->robot_info_.data_field_06.c_str());
    cvui::space(step);
    cvui::printf(0.4, 0xffffff, this->robot_info_.data_field_07.c_str());
    cvui::space(step);
    cvui::printf(0.4, 0xffffff, this->robot_info_.data_field_08.c_str());
    cvui::space(step);
    cvui::printf(0.4, 0xffffff, this->robot_info_.data_field_09.c_str());
    cvui::space(step);
    cvui::printf(0.4, 0xffffff, this->robot_info_.data_field_10.c_str());
    cvui::endColumn();
    cvui::endRow();
    /*  --------------------  */

    /*  ----------  teleoperation buttons  ----------  */
    cvui::beginRow(frame, x_win_button, y_win_button, button_win_width,
                   button_win_height);
      cvui::beginColumn(button_width, button_win_height);
        cvui::space(button_height + inter_button_space);
        if (cvui::button(button_width, button_height, "left")) {
          this->cmd_vel_.angular.z += 0.1;
        }
      cvui::endColumn();
      cvui::space(inter_button_space);
      cvui::beginColumn(button_width, button_win_height);
        if (cvui::button(button_width, button_height, "forward")) {
          this->cmd_vel_.linear.x += 0.1;
        }
        cvui::space(inter_button_space);
        if (cvui::button(button_width, button_height, "stop")) {
          this->cmd_vel_.linear.x = 0;
          this->cmd_vel_.angular.z = 0;
        }
        cvui::space(inter_button_space);
        if (cvui::button(button_width, button_height, "backward")) {
          this->cmd_vel_.linear.x += -0.1;
        }
      cvui::endColumn();
      cvui::space(inter_button_space);
      cvui::beginColumn(button_width, button_win_height);
        cvui::space(button_height + inter_button_space);
        if (cvui::button(button_width, button_height, "right")) {
          this->cmd_vel_.angular.z += -0.1;
        }
      cvui::endColumn();
    cvui::endRow();

    this->cmd_vel_pub_.publish(this->cmd_vel_);
    /*  --------------------  */



    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(inter_button_space) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
  }
}