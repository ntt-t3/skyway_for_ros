//
// Created by nakakura on 4/3/23.
//

#ifndef CONTROL_SERVICE_H
#define CONTROL_SERVICE_H

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "skyway/srv/sky_way_control.hpp"

class ControlService : public rclcpp::Node {
public:
  ControlService(const std::string &name_space = "",
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void set_callback(std::function<std::string(std::string)> callback);

private:
  rclcpp::Service<skyway::srv::SkyWayControl>::SharedPtr srv_;
  std::unique_ptr<std::function<std::string(std::string)>> callback_;

  void handle_service_(
      const std::shared_ptr<skyway::srv::SkyWayControl::Request> request,
      const std::shared_ptr<skyway::srv::SkyWayControl::Response> response);
};

#endif // CONTROL_SERVICE_H
