//
// Created by nakakura on 4/10/23.
//

#include "events_service.h"

void EventsService::handleService_(
    const std::shared_ptr<skyway::srv::SkyWayEvents::Request>,
    const std::shared_ptr<skyway::srv::SkyWayEvents::Response> response) {
  response->response = (*callback_)();
}

EventsService::EventsService(const std::string &name_space,
                             const rclcpp::NodeOptions &options)
    : Node("skyway_events", name_space, options) {
  using namespace std::placeholders;

  // Set default callback function
  callback_ =
      std::make_unique<std::function<std::string()>>([]() -> std::string {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Callback function is not set yet.");
        return "CALLBACK NOT SET YET";
      });

  srv_ = create_service<skyway::srv::SkyWayEvents>(
      "srv_test", std::bind(&EventsService::handleService_, this, _1, _2));
}

void EventsService::set_callback(std::function<std::string()> callback) {
  callback_ = std::make_unique<std::function<std::string()>>(callback);
}
