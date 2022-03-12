#include "../../src/infra/source_impl.h"

#include <fruit/fruit.h>
#include <gtest/gtest.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <unordered_map>

using boost::asio::ip::udp;
using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

// di.cpp内のものと特に変わらない
Component<SourceFactory> getSourceComponent() {
  return createComponent().bind<Source, DataChannelSourceImpl>();
}

unsigned short get_port() {
  unsigned short port;
  {
    io_service io_service;
    udp::socket socket(io_service);
    socket.open(boost::asio::ip::udp::v4());
    socket.bind(udp::endpoint(udp::v4(), 0));
    port = socket.local_endpoint().port();
    socket.close();
  }
  return port;
}

void publish_data(std::string topic_name, int data_len) {
  ros::NodeHandle nh;
  // topicにデータを流し込むスレッド
  ros::Publisher chatter_pub_ =
      nh.advertise<std_msgs::UInt8MultiArray>(topic_name, 10000);
  ros::Rate loop_rate(100);

  for (int i = 0; i < data_len; i++) {
    std_msgs::UInt8MultiArray array;
    std::vector<char> send_data(i + 1, i);
    array.data.insert(array.data.end(), &send_data[0],
                      &send_data[send_data.size()]);
    chatter_pub_.publish(array);
    loop_rate.sleep();
  }
}

void udp_recv(unsigned short port, int recv_data_len, bool &is_received,
              std::vector<std::vector<char>> &recv_data_array) {
  int sock;
  struct sockaddr_in addr;
  struct sockaddr_in senderinfo;
  socklen_t addrlen;
  char buf[2048];
  char senderstr[16];
  int n;
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;
  bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  for (int i = 0; i < recv_data_len; i++) {
    memset(buf, 0, sizeof(buf));
    addrlen = sizeof(senderinfo);
    n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&senderinfo,
                 &addrlen);
    std::vector<char> dest(n);
    copy(buf, buf + n, dest.begin());
    recv_data_array.push_back(dest);
  }
  close(sock);
  is_received = true;
}

// 間違えて何度か連続でstart, stopしても問題ないことをチェックする
TEST(TestSuite, start_and_stop_many_times) {
  ros::NodeHandle nh;
  // データを受信したかどうかを示すフラグ
  // このフラグが立たないとテストを抜けない
  bool is_received = false;
  // 受信したデータを格納する配列
  std::vector<std::vector<char>> recv_data_array;

  unsigned short port = get_port();

  // source objectを作成
  Injector<SourceFactory> injector(getSourceComponent);
  SourceFactory sourceFactory(injector);
  auto source = sourceFactory("/chatter", udp::endpoint(udp::v4(), 0),
                              udp::endpoint(udp::v4(), port));

  // 間違えて5回startしてしまった場合
  source->Start();
  source->Start();
  source->Start();
  source->Start();
  source->Start();

  // データをpublishしてDataSourceに流し込むスレッド
  std::thread sending_thread([&] { publish_data("/chatter", 100); });

  // DataSourceからUDPを受信するスレッド
  std::thread receiving_thread(
      [&] { udp_recv(port, 100, is_received, recv_data_array); });

  // データの受け渡しが完了するまでspinする
  ros::Rate loop_rate(100);
  while (!ros::isShuttingDown() && !is_received) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  sending_thread.join();
  receiving_thread.join();
  // 間違えて3回stopしてしまった場合
  source->Stop();
  source->Stop();
  source->Stop();

  for (auto vec : recv_data_array) {
    ASSERT_EQ((int)vec.data()[0] + 1, vec.size());
  }
}

// 実際に利用する際にはunordered_mapに入れて使うので、入れた状態でテストする
TEST(TestSuite, datasource_in_map) {
  ros::NodeHandle nh;
  // データを受信したかどうかを示すフラグ
  // このフラグが立たないとテストを抜けない
  bool is_received = false;
  // 受信したデータを格納する配列
  static std::vector<std::vector<char>> recv_data_array;

  unsigned short port = get_port();

  // source objectを作成
  Injector<SourceFactory> injector(getSourceComponent);
  SourceFactory sourceFactory(injector);
  auto source = sourceFactory("/chatter", udp::endpoint(udp::v4(), 0),
                              udp::endpoint(udp::v4(), port));

  // source objectをmapで管理
  std::unordered_map<std::string, std::unique_ptr<Source>> map;
  map.emplace("hoge", std::move(source));
  map.at("hoge")->Start();

  // データをpublishしてDataSourceに流し込むスレッド
  std::thread sending_thread([&] { publish_data("/chatter", 100); });

  // DataSourceからUDPを受信するスレッド
  std::thread receiving_thread(
      [&] { udp_recv(port, 100, is_received, recv_data_array); });

  // データの受け渡しが完了するまでspinする
  ros::Rate loop_rate(100);
  while (!ros::isShuttingDown() && !is_received) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  sending_thread.join();
  receiving_thread.join();
  map.at("hoge")->Stop();

  for (auto vec : recv_data_array) {
    ASSERT_EQ((int)vec.data()[0] + 1, vec.size());
  }
}

// 実際に利用する際にはunordered_mapに入れて使うので、入れた状態でテストする
TEST(TestSuite, datasource_insert_map_after_start) {
  ros::NodeHandle nh;
  // データを受信したかどうかを示すフラグ
  // このフラグが立たないとテストを抜けない
  bool is_received = false;
  // 受信したデータを格納する配列
  static std::vector<std::vector<char>> recv_data_array;

  unsigned short port = get_port();

  // source objectを作成
  Injector<SourceFactory> injector(getSourceComponent);
  SourceFactory sourceFactory(injector);
  auto source = sourceFactory("/chatter", udp::endpoint(udp::v4(), 0),
                              udp::endpoint(udp::v4(), port));
  source->Start();

  // source objectをmapで管理
  std::unordered_map<std::string, std::unique_ptr<Source>> map;
  map.emplace("hoge", std::move(source));

  // データをpublishしてDataSourceに流し込むスレッド
  std::thread sending_thread([&] { publish_data("/chatter", 100); });

  // DataSourceからUDPを受信するスレッド
  std::thread receiving_thread(
      [&] { udp_recv(port, 100, is_received, recv_data_array); });

  // データの受け渡しが完了するまでspinする
  ros::Rate loop_rate(100);
  while (!ros::isShuttingDown() && !is_received) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  sending_thread.join();
  receiving_thread.join();
  map.at("hoge")->Stop();

  for (auto vec : recv_data_array) {
    ASSERT_EQ((int)vec.data()[0] + 1, vec.size());
  }
}

// 100個のデータをUDPで送信するが、相手側が3つまでしか受け取らないケース
// 送信エラーなどを出さずに送信終了し、相手側は3つ受信していることを確認する
TEST(TestSuite, datachannel_early_exit) {
  ros::NodeHandle nh;
  // データを受信したかどうかを示すフラグ
  // このフラグが立たないとテストを抜けない
  bool is_received = false;
  // 受信したデータを格納する配列
  static std::vector<std::vector<char>> recv_data_array;

  unsigned short port = get_port();

  // source objectを作成
  Injector<SourceFactory> injector(getSourceComponent);
  SourceFactory sourceFactory(injector);
  auto source = sourceFactory("/chatter", udp::endpoint(udp::v4(), 0),
                              udp::endpoint(udp::v4(), port));

  source->Start();

  // データをpublishしてDataSourceに流し込むスレッド
  std::thread sending_thread([&] { publish_data("/chatter", 100); });

  // DataSourceからUDPを受信するスレッド
  // 100発送られるところ3発しか受けない
  std::thread receiving_thread(
      [&] { udp_recv(port, 3, is_received, recv_data_array); });

  ros::Rate loop_rate(100);
  while (!ros::isShuttingDown() && !is_received) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  sending_thread.join();
  receiving_thread.join();
  source->Stop();

  for (auto vec : recv_data_array) {
    ASSERT_EQ((int)vec.data()[0] + 1, vec.size());
  }

  // 3つ分のデータを受け取っているはずである
  ASSERT_EQ(recv_data_array.size(), 3);
}
