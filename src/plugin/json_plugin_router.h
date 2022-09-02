//
// Created by nakakura on 22/08/31.
//

#ifndef SKYWAY_PLUGIN_UDP_PIPE_STRING_PLUGIN_ROUTER_H
#define SKYWAY_PLUGIN_UDP_PIPE_STRING_PLUGIN_ROUTER_H

#include <fruit/fruit.h>
#include <pluginlib/class_loader.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <ros/ros.h>

#include "../socket/udp_socket.h"
#include "plugin_router.h"
#include "skyway_plugin/skyway_plugin.h"

using namespace rapidjson;
using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class JsonPluginRouter : public PluginRouter {
 private:
  ros::NodeHandle nh_;
  pluginlib::ClassLoader<skyway_plugin::SkyWayJsonPlugin> plugin_loader_;
  std::vector<boost::shared_ptr<skyway_plugin::SkyWayJsonPlugin>> plugins_;
  udp::endpoint target_socket_;
  std::unique_ptr<Socket> socket_;
  XmlRpc::XmlRpcValue config_;

  void observe_socket(std::vector<uint8_t> data);
  void observe_plugins(std::shared_ptr<rapidjson::Document> document);

 public:
  JsonPluginRouter() = delete;
  INJECT(JsonPluginRouter(ASSISTED(XmlRpc::XmlRpcValue) config,
                          ASSISTED(udp::endpoint) target_socket,
                          SocketFactory factory));
  ~JsonPluginRouter();

  virtual PluginResult TryStart() override;
};

using JsonPluginRouterFactory = std::function<std::unique_ptr<PluginRouter>(
    XmlRpc::XmlRpcValue, udp::endpoint)>;

#endif  // SKYWAY_PLUGIN_UDP_PIPE_STRING_PLUGIN_ROUTER_H
