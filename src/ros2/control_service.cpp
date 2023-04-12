//
// Created by nakakura on 4/3/23.
//

#include "control_service.h"

void ControlService::handle_service_(
    const std::shared_ptr<skyway::srv::SkyWayControl::Request> request,
    const std::shared_ptr<skyway::srv::SkyWayControl::Response> response) {
  response->response = (*callback_)(request->request);
}

ControlService::ControlService(const std::string &name_space,
                               const rclcpp::NodeOptions &options)
    : Node("skyway_control", name_space, options) {
  using namespace std::placeholders;

  // Set default callback function
  callback_ = std::make_unique<std::function<std::string(std::string)>>(
      [](std::string) -> std::string {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Callback function is not set yet.");
        return "CALLBACK NOT SET YET";
      });

  // Create service
  srv_ = this->create_service<skyway::srv::SkyWayControl>(
      "srv_test", std::bind(&ControlService::handle_service_, this, _1, _2));
}

void ControlService::set_callback(
    std::function<std::string(std::string)> callback) {
  callback_ =
      std::make_unique<std::function<std::string(std::string)>>(callback);
}
