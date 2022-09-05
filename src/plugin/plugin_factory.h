//
// Created by nakakura on 22/09/02.
//

#ifndef SKYWAY_PLUGIN_FACTORY_H
#define SKYWAY_PLUGIN_FACTORY_H

#include <fruit/fruit.h>

#include "binary_plugin_router.h"
#include "json_plugin_router.h"
#include "string_plugin_router.h"

using fruit::Annotated;
using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

/*
class PluginFactory {
 private:
 public:
  virtual ~PluginFactory() = default;
  virtual int Test() { return 0; }
  virtual void say() { ROS_ERROR("Hello"); }
};

class PluginFactoryImpl : public PluginFactory {
 public:
  PluginFactoryImpl() = delete;
  INJECT(PluginFactoryImpl(
      ANNOTATED(BinaryAnnotation, PluginRouterFactory) binary_factory,
      ANNOTATED(JsonAnnotation, PluginRouterFactory) json_factory,
      ANNOTATED(StringAnnotation, PluginRouterFactory) string_factory)) {
    ros::NodeHandle nh;
    XmlRpc::XmlRpcValue config;
    nh.getParam("/test_node/invalid_string_config", config);
    auto source = string_factory(config, udp::endpoint(udp::v4(), 0));
    auto result = source->TryStart();
    ROS_ERROR("constructor %s", result.error_message.c_str());
  }
  ~PluginFactoryImpl() {}
  virtual int Test() override { return 100; }
  void say() override { ROS_ERROR("Bonjour in sub"); }
};

Component<PluginFactory> getPluginFactoryComponent();
*/

#endif  // SKYWAY_PLUGIN_FACTORY_H
