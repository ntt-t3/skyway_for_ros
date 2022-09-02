//
// Created by nakakura on 22/08/19.
//

#ifndef SKYWAY_PLUGIN_UDP_PIPE_DATA_CHANNEL_PIPE_H
#define SKYWAY_PLUGIN_UDP_PIPE_DATA_CHANNEL_PIPE_H

#include <fruit/fruit.h>
#include <ros/ros.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <thread>

#include "socket.h"
#include "std_msgs/UInt8MultiArray.h"

using boost::asio::io_service;
using boost::asio::ip::address;
using boost::asio::ip::udp;
using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class UdpSocket : public Socket {
 private:
  std::shared_ptr<std::function<void(std::vector<uint8_t>)>> callback_;

  // WebRTC GWのData Portの情報が格納される
  udp::endpoint target_socket_;
  // 送信エラーを管理する変数
  boost::system::error_code err_;
  // WebRTC GWからデータを受け取るためのポート情報とsocket
  udp::endpoint local_endpoint_{};
  std::unique_ptr<boost::asio::io_service> io_service_{new io_service()};
  std::unique_ptr<udp::socket> socket_{
      std::make_unique<udp::socket>(*io_service_.get())};
  // WebRTC GWからデータを受信するスレッド
  std::unique_ptr<std::thread> recv_thread_{nullptr};
  // WebRTC GWから受信したUDPペイロードを格納するためのオブジェクト
  // 1500(MTU) - 14(Ethernet Header) - 40(IPv6 Header) - 12(SCTP Header)
  // = 1434bytesなので、Max 1434で用意する。
  boost::array<uint8_t, 1434> recv_buffer_;
  // start, stopメソッドの重複コールを避けるため利用するフラグ
  bool is_running_ = false;

  void send_handler(const boost::system::error_code &error, std::size_t len);
  void receive_handler(const boost::system::error_code &error,
                       size_t bytes_transferred);
  void wait_for_packets();

 public:
  // デフォルトコンストラクタは削除
  UdpSocket() = delete;
  INJECT(UdpSocket(
      ASSISTED(udp::endpoint) target_socket,
      ASSISTED(std::shared_ptr<std::function<void(std::vector<uint8_t>)>>)
          callback))
      : target_socket_(target_socket), callback_(callback) {}
  // デストラクタでは、PublisherとUDPポートの開放を行う
  ~UdpSocket() { Stop(); }
  // moveコンストラクタを追加
  UdpSocket(UdpSocket &&) = default;
  // WebRTC GWからのデータ受信とPublishを開始する
  // 重複コールは許容するが、stop後の再開はサポートしない(未定義動作)
  virtual void Start() override;
  // WebRTC GWからのデータ受信とPublishを停止する
  // 重複コールは許容する
  virtual void Stop() override;
  // 内部で保持しているソケットのポート番号を取得する
  virtual unsigned short Port() override;
  // socketからデータを送信する。非同期送信される
  virtual void SendData(std::vector<uint8_t>) override;
};

using SocketFactory = std::function<std::unique_ptr<Socket>(
    udp::endpoint, std::shared_ptr<std::function<void(std::vector<uint8_t>)>>)>;

fruit::Component<SocketFactory> getSocketComponent();

#endif  // SKYWAY_PLUGIN_UDP_PIPE_DATA_CHANNEL_PIPE_H
