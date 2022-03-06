#include <ros/ros.h>

typedef void (*func_ptr_t)(char*);

extern "C" {
void log_debug_c(char* message) { ROS_DEBUG("%s", message); }

void log_info_c(char* message) { ROS_INFO("%s", message); }

void log_warn_c(char* message) { ROS_WARN("%s", message); }

void log_err_c(char* message) { ROS_ERROR("%s", message); }

void register_logger(func_ptr_t debug, func_ptr_t info, func_ptr_t warn,
                     func_ptr_t error);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "skyway");
  ros::AsyncSpinner spinner(1);

  spinner.start();

  register_logger(log_debug_c, log_info_c, log_warn_c, log_err_c);

  spinner.stop();
  return 0;
}
