#include <ros/ros.h>

#include "presentation/control_service.h"
#include "router.h"

using void_char_func = void (*)(char*);
using void_double_func = void (*)(double);
using void_void_func = void (*)();
using bool_void_func = bool (*)();

extern "C" {
typedef struct {
  bool flag;
  void* handler;
} run_response_t;

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
run_response_t run();
void join_handler(void* handler);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "skyway");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Rust側からC++側の関数を呼び出すためのセッティング
  register_logger(log_debug_c, log_info_c, log_warn_c, log_err_c);
  register_program_state(is_ok_c, is_shutting_down_c, ros_sleep_c,
                         wait_for_shutdown_c);
  // Rust側の処理開始
  run_response_t response = run();

  if (response.flag) {
    // Rust側の処理が正常に開始した
    ROS_DEBUG("program starts successfully");
    Injector<Router> apiInjector(getRouterComponent);
    std::shared_ptr<Router> router = apiInjector.get<std::shared_ptr<Router>>();
    router->Start();
    // メインスレッドはここで止めてあとはSeviceとActionからの処理を待ち受け続ける
    ros::waitForShutdown();
  } else {
    // Rust側の処理が正常に開始しなかった
    // registerを忘れているケースなので通常発生しない
    ROS_ERROR("some errors occurred");
  }

  spinner.stop();
  ros::shutdown();
  // Rust側のthreadが終了するまで待機する
  if (response.flag) {
    ROS_DEBUG("Waiting for Rust side thread to finish");
    join_handler(response.handler);
  }
  return 0;
}
