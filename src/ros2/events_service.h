//
// Created by nakakura on 4/10/23.
//

#ifndef SKYWAY_EVENTS_SERVICE_H
#define SKYWAY_EVENTS_SERVICE_H

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "skyway/srv/sky_way_events.hpp"

class EventsService : public rclcpp::Node {
public:
  EventsService(const std::string &name_space = "",
                const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void set_callback(std::function<std::string()> callback);

private:
  void handleService_(
      const std::shared_ptr<skyway::srv::SkyWayEvents::Request> request,
      const std::shared_ptr<skyway::srv::SkyWayEvents::Response> response);

private:
  rclcpp::Service<skyway::srv::SkyWayEvents>::SharedPtr srv_;
  std::unique_ptr<std::function<std::string()>> callback_;
};

#endif // SKYWAY_EVENTS_SERVICE_H
