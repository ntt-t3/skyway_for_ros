//
// Created by nakakura on 22/08/29.
//
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <skyway/skyway_plugin.h>
#include <skyway/string_send_recv.h>

string_send_recv::StringSendRecv::StringSendRecv() {
  ROS_INFO("string_send_recv plugin loaded");
}

void string_send_recv::StringSendRecv::Initialize(
    std::shared_ptr<rapidjson::Document> parameter,
    std::shared_ptr<std::function<void(std::string)>> callback) {
  callback_ = callback;
  (*callback_)("message from string_send_recv::StringSendRecv");
}

void string_send_recv::StringSendRecv::Execute(std::string data) {
  ROS_INFO("recv: %s", data.c_str());
}

void string_send_recv::StringSendRecv::Shutdown() {}
