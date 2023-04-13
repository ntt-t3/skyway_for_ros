//
// Created by nakakura on 22/08/29.
//
#include <skyway/binary_loopback.h>

binary_loopback::BinaryLoopback::BinaryLoopback()
    : node_(rclcpp::Node::make_shared("skyway_node")) {
  RCLCPP_INFO(node_->get_logger(), "binary_loopback plugin loaded");
}

binary_loopback::BinaryLoopback::~BinaryLoopback() {
  RCLCPP_INFO(node_->get_logger(), "binary_loopback plugin exited");
}

void binary_loopback::BinaryLoopback::Initialize(
    std::shared_ptr<rapidjson::Document>,
    std::shared_ptr<std::function<void(std::vector<uint8_t>)>> callback) {
  callback_ = callback;
}

void binary_loopback::BinaryLoopback::Execute(std::vector<uint8_t> data) {
  (*callback_)(data);
}

void binary_loopback::BinaryLoopback::Shutdown() {}
