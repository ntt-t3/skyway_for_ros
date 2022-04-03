#include "control_service.h"
#include <ros/ros.h>

using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

// エンドユーザプログラムから与えられたメッセージをCallerに与え、レスポンスをServiceのClientに返す
bool ControlServiceImpl::callback(skyway::SkyWayControl::Request &req,
                                  skyway::SkyWayControl::Response &res) {
  ROS_ERROR("request %s", req.request.c_str());
  res.response = callback_(req.request);
  ROS_ERROR("response %s", res.response.c_str());
  return true;
}

// コンストラクタでは、サービス名とCaller内のSender Objectを受け取る
ControlServiceImpl::ControlServiceImpl(
    ASSISTED(std::string) name,
    ASSISTED(std::function<std::string(std::string)>) callback)
    : name_(name), callback_(callback) {
  service_ = nh_.advertiseService(name, &ControlServiceImpl::callback, this);
}
