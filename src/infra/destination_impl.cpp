#include "destination_impl.h"

using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

// WebRTC GWからのデータ受信とPublishを開始する
// 重複コールは許容するが、stop後の再開はサポートしない(未定義動作)
void DataChannelDestinationImpl::Start() {
  if (is_running_) return;

  if (!recv_thread_) {
    socket_->open(udp::v4());
    socket_->bind(local_endpoint_);

    ros::NodeHandle nh_;
    pub_ = nh_.advertise<std_msgs::UInt8MultiArray>(topic_name_, 10000);

    // io_service_->runは同期実行なので、別スレッドで行う
    recv_thread_.reset(new std::thread([&] {
      wait_for_packets();
      io_service_->run();
    }));
  }

  is_running_ = true;
}

// WebRTC GWからのデータ受信とPublishを停止する
// 重複コールは許容する
// 各種サービスを停止し、threadのポインタをクリアする
void DataChannelDestinationImpl::Stop() {
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
  if (pub_) {
    pub_.shutdown();
  }

  is_running_ = false;
}

// 内部で保持しているソケットのポート番号を取得する
unsigned short DataChannelDestinationImpl::Port() {
  if (socket_ && socket_->is_open()) {
    return socket_->local_endpoint().port();
  } else {
    return 0;
  }
}

// ========== private methods ==========
// boost::asioがUDPパケットを受信した際に呼ばれるコールバックメソッド
// 受信した瞬間にデータをpublishする
void DataChannelDestinationImpl::handle_receive(
    const boost::system::error_code &error, size_t bytes_transferred) {
  if (error) {
    if (error.message() != "Operation canceled") {
      std::cout << "Receive failed: " << error.message() << "\n";
    }
    return;
  }

  std_msgs::UInt8MultiArray array;
  array.data.insert(array.data.end(), &recv_buffer_[0],
                    &recv_buffer_[bytes_transferred]);
  pub_.publish(array);

  wait_for_packets();
}

// boost::asioで待受を開始する
void DataChannelDestinationImpl::wait_for_packets() {
  socket_.get();
  if (socket_) {
    socket_->async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        boost::bind(&DataChannelDestinationImpl::handle_receive, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
  }
}
