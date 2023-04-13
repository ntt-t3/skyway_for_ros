//
// Created by nakakura on 22/09/04.
//

/// Rust側から呼ばれた処理をハンドリングするためのクラス
#ifndef SKYWAY_FFI_BRIDGE_H
#define SKYWAY_FFI_BRIDGE_H

#include <fruit/fruit.h>
#include <ros/ros.h>

#include "common.h"
#include "ros_functions.h"

using fruit::Component;
using fruit::Injector;

// callback body
namespace {
std::function<void(int)> shutdown_handler;
std::function<void(char*, char*)> create_peer_callback_handler;
std::function<PluginLoadResult(char*, uint16_t, char*, char*)>
    create_data_callback_handler;
std::function<void(uint16_t)> data_connection_close_event_callback_handler;
}  // namespace

class CallbackFromRust {};

class CallbackFromRustImpl : public CallbackFromRust {
 private:
  void create_peer_callback(char* peer_id, char* token);
  PluginLoadResult create_data_connection_callback(char*, uint16_t, char*,
                                                   char*);
  void delete_data_connection_callback(uint16_t);

  std::shared_ptr<Router> router_;

 public:
  CallbackFromRustImpl() = delete;
  INJECT(CallbackFromRustImpl(std::shared_ptr<Router> router));
  virtual ~CallbackFromRustImpl() {}
};

Component<CallbackFromRust> getCallbackFromRustComponent();

#endif  // SKYWAY_FFI_BRIDGE_H
