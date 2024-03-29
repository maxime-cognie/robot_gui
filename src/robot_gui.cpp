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
  this->cmd_vel_pub_ =
      this->nh_->advertise<geometry_msgs::Twist>(this->cmd_vel_topic_, 1);
  this->cmd_vel_.linear.x = 0;
  this->cmd_vel_.angular.z = 0;
}

void RobotGui::robotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  this->robot_info_ = *msg;
}

void RobotGui::run() {

  // size of the main window
  const int mw_width = 600;
  const int mw_height = 1000;

  cv::Mat frame = cv::Mat(mw_height, mw_width, CV_8UC3);

  // color
  const unsigned int white = 0xffffff;

  // line separation
  const int step = 5;
  // position and size of the general info window
  const int x_win_info = (mw_width - 435) / 2; // center the window
  const int y_win_info = 20;
  const int win_info_width = 435;
  const int win_info_height = 100;

  // teleoparation buttons info
  const int x_win_button = 100;
  const int y_win_button = y_win_info + win_info_height + 40;
  const int button_win_width = mw_width - 2 * x_win_button;
  const int button_win_height = 280;
  const int inter_button_space = 20;
  const int button_width = std::lround((button_win_width - 2 * inter_button_space) / 3);
  const int button_height = 80;

  //
  const int curr_vel_win_width = (mw_width - 60) / 2;
  const int curr_vel_win_height = 40;
  const int x_win_current_lin = 20;
  const int x_win_current_ang = mw_width - curr_vel_win_width - 20;
  const int y_win_current_vel = y_win_button + 300;


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
    cvui::beginColumn(win_info_width / 2, win_info_height, step);
    cvui::printf(0.4, white, this->robot_info_.data_field_01.c_str());
    cvui::printf(0.4, white, this->robot_info_.data_field_02.c_str());
    cvui::printf(0.4, white, this->robot_info_.data_field_03.c_str());
    cvui::printf(0.4, white, this->robot_info_.data_field_04.c_str());
    cvui::printf(0.4, white, this->robot_info_.data_field_05.c_str());
    cvui::endColumn();
    cvui::beginColumn(win_info_width / 2, win_info_height, step);
    cvui::printf(0.4, white, this->robot_info_.data_field_06.c_str());
    cvui::printf(0.4, white, this->robot_info_.data_field_07.c_str());
    cvui::printf(0.4, white, this->robot_info_.data_field_08.c_str());
    cvui::printf(0.4, white, this->robot_info_.data_field_09.c_str());
    cvui::printf(0.4, white, this->robot_info_.data_field_10.c_str());
    cvui::endColumn();
    cvui::endRow();
    /*  --------------------  */

    /*  ----------  teleoperation buttons  ----------  */
    cvui::beginRow(frame, x_win_button, y_win_button, button_win_width,
                   button_win_height, inter_button_space);
      cvui::beginColumn(button_width, button_win_height, inter_button_space);
        cvui::space(button_height);
        if (cvui::button(button_width, button_height, "left")) {
          this->cmd_vel_.angular.z += 0.1;
        }
      cvui::endColumn();
      //cvui::space(inter_button_space);
      cvui::beginColumn(button_width, button_win_height, inter_button_space);
        if (cvui::button(button_width, button_height, "forward")) {
          this->cmd_vel_.linear.x += 0.1;
        }
        if (cvui::button(button_width, button_height, "stop")) {
          this->cmd_vel_.linear.x = 0;
          this->cmd_vel_.angular.z = 0;
        }
        if (cvui::button(button_width, button_height, "backward")) {
          this->cmd_vel_.linear.x += -0.1;
        }
      cvui::endColumn();
      //cvui::space(inter_button_space);
      cvui::beginColumn(button_width, button_win_height, inter_button_space);
        cvui::space(button_height);
        if (cvui::button(button_width, button_height, "right")) {
          this->cmd_vel_.angular.z += -0.1;
        }
      cvui::endColumn();
    cvui::endRow();

    this->cmd_vel_pub_.publish(this->cmd_vel_);
    /*  --------------------  */

    /*  ----------  current position ----------  */
    cvui::window(frame, x_win_current_lin, y_win_current_vel, curr_vel_win_width, curr_vel_win_height, "linear velocity :");
    cvui::printf(frame, x_win_current_lin + 5, y_win_current_vel + 25, 0.4, white, "%.2f m/s", this->cmd_vel_.linear.x);
    cvui::window(frame, x_win_current_ang, y_win_current_vel, curr_vel_win_width, curr_vel_win_height, "angular velocity :");
    cvui::printf(frame, x_win_current_ang + 5, y_win_current_vel + 25, 0.4, white, "%.2f m/s", this->cmd_vel_.angular.z);
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