//
// Created by nakakura on 4/13/23.
//

#ifndef SKYWAY_RUST_FUNCTIONS_H
#define SKYWAY_RUST_FUNCTIONS_H

#include "common.h"

// C++側から呼び出されるRust側関数の定義
extern "C" {
void register_callbacks(Function& functions);
char* call_service(const char* message);
char* receive_events();
void release_string(char* message);
void create_peer_callback(char* peer_id, char* token);
void peer_deleted_callback();
PluginLoadResult create_data_callback(char* parameter);
void data_connection_close_event_callback(char* data_connection_id);

void register_logger(void_char_func debug, void_char_func info,
                     void_char_func warn, void_char_func error);
void register_program_state(bool_void_func is_running_c,
                            bool_void_func is_shutting_down_c,
                            void_double_func sleep_c,
                            void_void_func wait_for_shutdown_c,
                            void_void_func shutdown_c);
run_response_t run();
void join_handler(void* handler);

void print_string(char* message);
};

#endif  // SKYWAY_RUST_FUNCTIONS_H
