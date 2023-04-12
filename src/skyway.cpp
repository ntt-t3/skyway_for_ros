#include <rclcpp/rclcpp.hpp>

#include "ros2/events_service.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EventsService>();

  // Set callback function
  node->set_callback([]() -> std::string { return "Hello, world!"; });

  // Run service
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
