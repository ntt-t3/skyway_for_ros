//
// Created by nakakura on 22/08/31.
//

#ifndef SKYWAY_PLUGIN_UDP_PIPE_STRING_PLUGIN_ROUTER_H
#define SKYWAY_PLUGIN_UDP_PIPE_STRING_PLUGIN_ROUTER_H

#include <fruit/fruit.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include "../socket/udp_socket.h"
#include "plugin_router.h"
#include "skyway_plugin/skyway_plugin.h"

using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class StringPluginRouter : public PluginRouter {
 private:
  ros::NodeHandle nh_;
  pluginlib::ClassLoader<skyway_plugin::SkyWayStringPlugin> plugin_loader_;
  std::vector<boost::shared_ptr<skyway_plugin::SkyWayStringPlugin>> plugins_;
  udp::endpoint target_socket_;
  std::unique_ptr<Socket> socket_;
  XmlRpc::XmlRpcValue config_;

  void observe_socket(std::vector<uint8_t> data);
  void observe_plugins(std::string message);

 public:
  StringPluginRouter() = delete;
  INJECT(StringPluginRouter(ASSISTED(XmlRpc::XmlRpcValue) config,
                            ASSISTED(udp::endpoint) target_socket,
                            SocketFactory factory));
  ~StringPluginRouter();

  virtual PluginResult TryStart() override;
};

Component<PluginRouterFactory> getStringPluginRouterComponent();

#endif  // SKYWAY_PLUGIN_UDP_PIPE_STRING_PLUGIN_ROUTER_H
