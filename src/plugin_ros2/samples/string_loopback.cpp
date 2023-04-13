//
// Created by nakakura on 22/08/29.
//
#include <skyway/string_loopback.h>

string_loopback::StringLoopback::StringLoopback()
    : node_(rclcpp::Node::make_shared("skyway_node")) {
  RCLCPP_INFO(node_->get_logger(), "string_loopback plugin loaded");
}

string_loopback::StringLoopback::~StringLoopback() {
  RCLCPP_INFO(node_->get_logger(), "string_loopback plugin exited");
}

void string_loopback::StringLoopback::Initialize(
    std::shared_ptr<rapidjson::Document>,
    std::shared_ptr<std::function<void(std::string)>> callback) {
  callback_ = callback;
}

void string_loopback::StringLoopback::Execute(std::string data) {
  (*callback_)(data);
}

void string_loopback::StringLoopback::Shutdown() {}
