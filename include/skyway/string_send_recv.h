//
// Created by nakakura on 22/08/31.
//

#ifndef SKYWAY_PLUGIN_STRING_SEND_RECV_H
#define SKYWAY_PLUGIN_STRING_SEND_RECV_H

#include <skyway/skyway_plugin.h>

#include <thread>

namespace string_send_recv {
class StringSendRecv : public skyway_plugin::SkyWayStringPlugin {
 private:
  std::shared_ptr<std::function<void(std::string)>> callback_;
  std::thread loop_thread_;
  bool is_running_;

  void service_thread();

 public:
  StringSendRecv();
  virtual void Initialize(
      std::shared_ptr<rapidjson::Document> parameter,
      std::shared_ptr<std::function<void(std::string)>> callback) override;
  virtual void Execute(std::string data) override;
  virtual void Shutdown() override;
};
};  // namespace string_send_recv

#endif  // SKYWAY_PLUGIN_STRING_SEND_RECV_H
