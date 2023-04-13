//
// Created by nakakura on 22/08/29.
//
#include <skyway/json_loopback.h>

json_loopback::JsonLoopback::JsonLoopback()
    : node_(rclcpp::Node::make_shared("skyway_node")) {
  RCLCPP_INFO(node_->get_logger(), "json_loopback plugin loaded");
}

json_loopback::JsonLoopback::~JsonLoopback() {
  RCLCPP_INFO(node_->get_logger(), "json_loopback plugin exited");
}

void json_loopback::JsonLoopback::Initialize(
    std::shared_ptr<rapidjson::Document>,
    std::shared_ptr<std::function<void(std::shared_ptr<rapidjson::Document>)>>
        callback) {
  callback_ = callback;
}

void json_loopback::JsonLoopback::Execute(
    std::shared_ptr<rapidjson::Document> data) {
  (*callback_)(data);
}

void json_loopback::JsonLoopback::Shutdown() {}
