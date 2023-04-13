//
// Created by nakakura on 4/13/23.
//

#ifndef SKYWAY_COMMON_H
#define SKYWAY_COMMON_H

// C++側から呼び出されるRust側関数の定義
extern "C" {
struct PluginLoadResult {
  bool is_success;
  uint16_t port;
  const char* error_message;
};

using void_char_func = void (*)(char*);
using void_uint16_func = void (*)(uint16_t);
using void_double_func = void (*)(double);
using void_void_func = void (*)();
using bool_void_func = bool (*)();
using void_char_char_func = void (*)(char*, char*);
using void_char_func = void (*)(char*);
using void_void_func = void (*)();
using plugin_topicparam_func = PluginLoadResult (*)(char*, uint16_t, char*,
                                                    char*);

struct Function {
  void_char_char_func create_peer_callback;
  void_void_func peer_deleted_callback;
  plugin_topicparam_func create_data_callback;
  void_uint16_func data_connection_deleted_callback;
  void_char_func release_string_callback;
};

struct run_response_t {
  bool flag;
  void* handler;
};
}

#endif  // SKYWAY_COMMON_H
