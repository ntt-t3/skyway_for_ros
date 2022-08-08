// DataChannelから得られたデータをエンドユーザに返すオブジェクトの実装

#ifndef SKYWAY_DESTINATION_IMPL_H
#define SKYWAY_DESTINATION_IMPL_H

#include <ros/ros.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <thread>

#include "../domain/entity.h"
#include "std_msgs/UInt8MultiArray.h"

using boost::asio::io_service;
using boost::asio::ip::address;
using boost::asio::ip::udp;
using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class DataChannelDestinationImpl : public Destination {
 private:
  // WebRTC GWから受け取ったデータをPublishするためのオブジェクト
  ros::Publisher pub_;
  std::string topic_name_;
  // WebRTC GWからデータを受け取るためのソケット
  std::unique_ptr<boost::asio::io_service> io_service_;
  std::unique_ptr<udp::socket> socket_;
  // WebRTC GWからデータを受信するスレッド
  std::unique_ptr<std::thread> recv_thread_{nullptr};
  // WebRTC GWから受信したUDPペイロードを格納するためのオブジェクト
  // 1500(MTU) - 14(Ethernet Header) - 40(IPv6 Header) - 12(SCTP Header)
  // = 1434bytesなので、Max 1434で用意する。
  boost::array<char, 1434> recv_buffer_;
  // start, stopメソッドの重複コールを避けるため利用するフラグ
  bool is_running_;
  udp::endpoint local_endpoint_;
  // WebRTC GWのData Portの情報が格納される
  udp::endpoint remote_endpoint_;

  void handle_receive(const boost::system::error_code &, size_t);
  void wait_for_packets();

 public:
  // デフォルトコンストラクタは削除
  DataChannelDestinationImpl() = delete;
  INJECT(DataChannelDestinationImpl(ASSISTED(std::string) topic_name,
                                    ASSISTED(udp::endpoint) local_endpoint))
      : topic_name_(topic_name),
        io_service_(new io_service()),
        local_endpoint_(local_endpoint),
        socket_(std::make_unique<udp::socket>(*io_service_.get())),
        is_running_(false) {
  }
  // デストラクタでは、PublisherとUDPポートの開放を行う
  ~DataChannelDestinationImpl() { Stop(); }
  // moveコンストラクタを追加
  DataChannelDestinationImpl(DataChannelDestinationImpl &&) = default;
  // WebRTC GWからのデータ受信とPublishを開始する
  // 重複コールは許容するが、stop後の再開はサポートしない(未定義動作)
  void Start() override;
  // WebRTC GWからのデータ受信とPublishを停止する
  // 重複コールは許容する
  void Stop() override;
  // 内部で保持しているソケットのポート番号を取得する
  unsigned short Port();
  // Topic名を取得
  virtual std::string TopicName() override { return topic_name_; }
};

#endif  // SKYWAY_DESTINATION_IMPL_H
