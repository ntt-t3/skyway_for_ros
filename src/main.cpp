#include <ros/ros.h>

using void_char_func = void (*)(char*);
using void_double_func = void (*)(double);
using void_void_func = void (*)();
using bool_void_func = bool (*)();

extern "C" {
// loggers
void log_debug_c(char* message) { ROS_DEBUG("%s", message); }
void log_info_c(char* message) { ROS_INFO("%s", message); }
void log_warn_c(char* message) { ROS_WARN("%s", message); }
void log_err_c(char* message) { ROS_ERROR("%s", message); }

// ros control functions
bool is_ok_c() { return ros::ok(); }
bool is_shutting_down_c() { return ros::isShuttingDown(); }
void ros_sleep_c(double dur) { ros::Duration(dur).sleep(); }
void wait_for_shutdown_c() { ros::waitForShutdown(); }

// in rust_module
void register_logger(void_char_func debug, void_char_func info,
                     void_char_func warn, void_char_func error);
void register_program_state(bool_void_func is_running_c,
                            bool_void_func is_shutting_down_c,
                            void_double_func sleep_c,
                            void_void_func wait_for_shutdown_c);
bool run();
void execute();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "skyway");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  register_logger(log_debug_c, log_info_c, log_warn_c, log_err_c);
  register_program_state(is_ok_c, is_shutting_down_c, ros_sleep_c,
                         wait_for_shutdown_c);
  bool flag = run();
  if (flag) {
    ROS_INFO("program runs successfully");
  } else {
    ROS_ERROR("some errors occurred");
  }

  ros::waitForShutdown();
  spinner.stop();
  return 0;
}
