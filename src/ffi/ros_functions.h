//
// Created by nakakura on 22/09/04.
//

#ifndef SKYWAY_FFI_H
#define SKYWAY_FFI_H

#include <ros/ros.h>
#include "../router.h"

// Rust側から呼び出されるC++側関数の定義
extern "C" {
// loggers
void log_debug_c(char* message);
void log_info_c(char* message);
void log_warn_c(char* message);
void log_err_c(char* message);

// ros control functions
bool is_ok_c();
bool is_shutting_down_c();
void ros_sleep_c(double dur);
void wait_for_shutdown_c();
void shutdown_c();
};

#endif  // SKYWAY_FFI_H
