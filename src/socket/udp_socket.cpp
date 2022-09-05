//
// Created by nakakura on 22/08/19.
//

#include "udp_socket.h"

// UDPの受信スレッドを開始する
// 重複コールは許容するが、stop後の再開はサポートしない(未定義動作)
void UdpSocket::Start() {
  if (is_running_) return;

  if (!recv_thread_) {
    socket_->open(udp::v4());
    socket_->bind(local_endpoint_);

    // io_service_->runは同期実行なので、別スレッドで行う
    recv_thread_.reset(new std::thread([&] {
      wait_for_packets();
      io_service_->run();
    }));
  }

  is_running_ = true;
}

// UDP受信スレッドを停止する
// 重複コールは許容する
// 各種サービスを停止し、threadのポインタをクリアする
void UdpSocket::Stop() {
  if (!is_running_) return;
  if (socket_) {
    socket_->cancel();
    socket_->close();
  }
  if (recv_thread_) {
    if (recv_thread_->joinable()) {
      recv_thread_->join();
    }
    recv_thread_.reset();
  }
  if (io_service_) {
    io_service_->stop();
    io_service_.reset();
  }

  is_running_ = false;
}

// 内部で保持しているソケットのポート番号を取得する
unsigned short UdpSocket::Port() {
  if (socket_ && socket_->is_open()) {
    return socket_->local_endpoint().port();
  } else {
    return 0;
  }
}

void UdpSocket::SendData(std::vector<uint8_t> vec) {
  std::string message(vec.begin(), vec.end());

  socket_->async_send_to(
      boost::asio::buffer(vec), target_socket_,
      boost::bind(&UdpSocket::send_handler, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

// ========== private methods ==========

// boost::asioがasyncでデータ送信完了した際のcallback
void UdpSocket::send_handler(const boost::system::error_code& error,
                             std::size_t len) {
  if (error.value() != boost::system::errc::success) {
    ROS_WARN("fail to send data. %s", error.message().c_str());
  }
}

// boost::asioがUDPパケットを受信した際に呼ばれるコールバックメソッド
void UdpSocket::receive_handler(const boost::system::error_code& error,
                                size_t bytes_transferred) {
  if (error) {
    if (error.message() != "Operation canceled") {
      ROS_ERROR("Receive failed: %s", error.message().c_str());
    }
    return;
  }

  // 受信データの処理開始前に非同期受信を再開しておく
  wait_for_packets();

  std::vector<uint8_t> vec;
  vec.insert(vec.end(), &recv_buffer_[0], &recv_buffer_[bytes_transferred]);
  (*callback_)(vec);
}

// boost::asioで待受を開始する
void UdpSocket::wait_for_packets() {
  socket_.get();
  if (socket_) {
    udp::endpoint remote_port;
    socket_->async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_port,
        boost::bind(&UdpSocket::receive_handler, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
  }
}

Component<SocketFactory> getUdpSocketComponent() {
  return createComponent().bind<Socket, UdpSocket>();
}
