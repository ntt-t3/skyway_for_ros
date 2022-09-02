//
// Created by nakakura on 22/08/25.
//

#ifndef SKYWAY_PLUGIN_UDP_PIPE_PLUGINROUTER_H
#define SKYWAY_PLUGIN_UDP_PIPE_PLUGINROUTER_H

struct PluginResult {
  bool is_success;
  uint16_t port;
  std::string error_message;
};

class PluginRouter {
 private:
 public:
  virtual ~PluginRouter() = default;
  virtual PluginResult TryStart() {
    return {.is_success = true, .error_message = ""};
  }
};

#endif  // SKYWAY_PLUGIN_UDP_PIPE_PLUGINROUTER_H
