#ifndef SKYWAY_SOURCE_IMPL_H
#define SKYWAY_SOURCE_IMPL_H

#include <ros/ros.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <regex>
#include <thread>

#include "../domain/entity.h"
#include "std_msgs/UInt8MultiArray.h"

using boost::asio::io_service;
using boost::asio::ip::address;
using boost::asio::ip::udp;
using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class DataChannelSourceImpl : public Source {
 private:
  // ROS Topicでデータを受け取る
  ros::Subscriber sub_;
  // ROS Topic名
  std::string topic_name_;
  // UDPでWebRTC GWにデータを送り込む
  std::unique_ptr<io_service> io_service_ptr_;
  std::unique_ptr<udp::socket> socket_;
  // WebRTC GWのData portの情報
  udp::endpoint remote_endpoint_;
  // 送信エラーを管理する変数
  boost::system::error_code err_;
  // start, stopメソッドの重複コールを避けるためのフラグ
  bool is_running_;

  // ROS TopicでSubscribeした際に呼ばれるコールバック
  void Callback(const std_msgs::UInt8MultiArray::ConstPtr &array);

 public:
  DataChannelSourceImpl() = delete;
  INJECT(DataChannelSourceImpl(ASSISTED(std::string) topic_name,
                               ASSISTED(udp::endpoint) remote_endpoint))
      : io_service_ptr_(new io_service()),
        socket_(new udp::socket(*io_service_ptr_, udp::endpoint(udp::v4(), 0))),
        remote_endpoint_(remote_endpoint),
        is_running_(false) {
    // Topic名としてDataConnectionIdを利用する。
    // DataConnectionIdは-を含むが、ROS Topic名に利用できないので
    // "-"を"_"に置換する
    topic_name_ = std::regex_replace(topic_name, std::regex("-"), "_");
  }
  ~DataChannelSourceImpl() { Stop(); }
  // ROS Topicでのデータ受信と、UDPでのデータ転送を開始する
  // 複数回呼ばれても問題ないが、stop後の再開はできない
  void Start() override;
  // ROS Topicでのデータ受信と、UDPでのデータ転送を停止する
  // 複数回呼ばれても問題ない
  void Stop() override;
  // Topic名を取得
  virtual std::string TopicName() override { return topic_name_; }
};

#endif  // SKYWAY_SOURCE_IMPL_H
