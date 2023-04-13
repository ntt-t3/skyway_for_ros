#include <skyway/skyway_plugin.h>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  pluginlib::ClassLoader<skyway_plugin::SkyWayStringPlugin> poly_loader(
      "skyway", "skyway_plugin::SkyWayStringPlugin");

  try {
    std::shared_ptr<skyway_plugin::SkyWayStringPlugin> triangle =
        poly_loader.createSharedInstance("string_loopback::StringLoopback");
  } catch (pluginlib::PluginlibException& ex) {
    printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
  }

  return 0;
}
