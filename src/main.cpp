#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "skyway");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("running");

  spinner.stop();
  return 0;
}
