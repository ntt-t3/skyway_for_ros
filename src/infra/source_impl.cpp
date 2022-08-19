// 概要
// SkyWay WebRTC Gateway(以下GW)で確立した
// DataChannelを通じて相手側に送信するデータは、
// エンドユーザプログラムからUDPでGWに送信する必要がある。
// エンドユーザプログラムがROS nodeである場合、
// ROS Topicで受け渡しする手段も利用可能な方が望ましいと考え、
// このクラスを準備する。
// /data APIのコール時に自動的にこのクラスのインスタンスが生成される。
// エンドユーザプログラムは、Topicでのデータ渡しをするか、このクラスを利用せず直接UDPでデータ送信するか選択可能である
//
// このクラスは以下の責務を持つ
// - ROS TopicでデータをSubscribeする。
//  データフォーマットはstd_msgs::UInt8MultiArrayを前提とする
//  (TODO:他のフォーマットでの転送の需要を確認する)
// - startメソッドコール時に、処理を開始し、
//   Subscriberで受信したデータをUDPでGWに送信する
// - stopメソッドコール時に、SubscriberとUDPでの送信を停止する
//   startメソッドのコールにより再開が可能である
//
// 生成と破棄のタイミング
// このクラスのインスタンスは1つのData portに紐づき、
// /data APIコール時に生成される。
//
// インスタンスは、Data Portのclose時に破棄されるべきである。
// Data Portの破棄タイミングは複数存在する
// - /data/delete APIのコール時
// - Data portが割り当てられているDataChannelのクローズ時
// - Data portが割り当てられているDataChannelを利用しているPeerの破棄時
// これらのタイミングを検知するのはこのクラスの利用者の責務である
// 破棄の前にはstopメソッドをコールするべきである

#include "source_impl.h"

using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

// ROS Topicでのデータ受信と、UDPでのデータ転送を開始する
// 複数回呼ばれても問題ないが、stop後の再開はできない
void DataChannelSourceImpl::Start() {
  if (is_running_) return;

  ros::NodeHandle nh_;
  ROS_ERROR("feedback");
  sub_ = nh_.subscribe("feedback_topic", 10000, &DataChannelSourceImpl::Callback,
                       this, ros::TransportHints().reliable().tcpNoDelay(true));

  is_running_ = true;
}

// ROS Topicでのデータ受信と、UDPでのデータ転送を停止する
void DataChannelSourceImpl::Stop() {
  if (!is_running_) return;
  if (socket_) {
    socket_->close();
  }
  if (sub_) {
    sub_.shutdown();
  }
  is_running_ = false;
}

// ROS Topicでデータを受信した際にUDPでデータを転送する
void DataChannelSourceImpl::Callback(
    const std_msgs::UInt8MultiArray::ConstPtr &array) {
  socket_->send_to(boost::asio::buffer(&array->data[0], array->data.size()),
                   remote_endpoint_, 0, err_);
  ROS_ERROR("%s", remote_endpoint_.address().to_string().c_str());
  ROS_ERROR("%d", remote_endpoint_.port());
}
