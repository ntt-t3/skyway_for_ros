//
// Created by nakakura on 4/12/23.
//

#ifndef SKYWAY_SKYAWY_PLUGIN_HPP
#define SKYWAY_SKYAWY_PLUGIN_HPP

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <functional>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

using namespace rapidjson;

namespace skyway_plugin {
class SkyWayBinaryPlugin {
 public:
  virtual void Initialize(
      std::shared_ptr<rapidjson::Document> parameter,
      std::shared_ptr<std::function<void(std::vector<uint8_t>)>> callback) = 0;
  virtual void Execute(std::vector<uint8_t> data) = 0;
  virtual void Shutdown() = 0;
  virtual ~SkyWayBinaryPlugin() {}

 protected:
  SkyWayBinaryPlugin() {}
};

class SkyWayStringPlugin {
 public:
  virtual void Initialize(
      std::shared_ptr<rapidjson::Document> parameter,
      std::shared_ptr<std::function<void(std::string)>> callback) = 0;
  virtual void Execute(std::string data) = 0;
  virtual void Shutdown() = 0;
  virtual ~SkyWayStringPlugin() {}

 protected:
  SkyWayStringPlugin() {}
};

class SkyWayJsonPlugin {
 public:
  virtual void Initialize(
      std::shared_ptr<rapidjson::Document> parameter,
      std::shared_ptr<std::function<void(std::shared_ptr<rapidjson::Document>)>>
          callback) = 0;
  virtual void Execute(std::shared_ptr<rapidjson::Document> document) = 0;
  virtual void Shutdown() = 0;
  virtual ~SkyWayJsonPlugin() {}

 protected:
  SkyWayJsonPlugin() {}
};
}  // namespace skyway_plugin

#endif  // SKYWAY_SKYAWY_PLUGIN_HPP
