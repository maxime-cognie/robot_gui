#include "robot_gui/robot_gui.h"
#include "ros/init.h"
#include "ros/node_handle.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_gui_node");
    ros::NodeHandle nh;
    RobotGui my_robot_gui(&nh);
    my_robot_gui.run();
    return 0;
}