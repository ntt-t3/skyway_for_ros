#include "../../src/infra/destination_impl.h"

#include <fruit/fruit.h>
#include <gtest/gtest.h>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <unordered_map>

#include "std_msgs/UInt8MultiArray.h"

using boost::asio::ip::udp;
using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

// 中身は特にdi.cppのものと変わらない
Component<DestinationFactory> getDestinationComponent() {
  return createComponent().bind<Destination, DataChannelDestinationImpl>();
}

void callback_internal(bool &flag, std::vector<std::vector<char>> &conatiner,
                       int counter,
                       const std_msgs::UInt8MultiArray::ConstPtr &array) {
  // 必要データ受信完了後は、データが届いても処理しない
  if (flag) return;

  std::vector<char> recv_data;
  copy(array->data.begin(), array->data.end(), back_inserter(recv_data));
  conatiner.push_back(recv_data);
  if (recv_data.size() == counter) {
    // 必要データ受信完了後
    flag = true;
  }
}

void send_udp(unsigned short port) {
  int sock;
  struct sockaddr_in addr;
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  addr.sin_port = htons(port);

  ros::Rate loop_rate(100);
  for (int i = 0; i < 100; i++) {
    std::vector<char> send_data(i + 1, i);
    sendto(sock, send_data.data(), sizeof(char) * (i + 1), 0,
           (struct sockaddr *)&addr, sizeof(addr));
    loop_rate.sleep();
  }
}

// 何回か連続してstart, stopを呼んだときのテスト
TEST(TestSuite, destination_start_and_stop_many_times) {
  // データの受信を完了し、テストを抜けて良くなったらtrueにするフラグ
  bool is_received = false;
  // 受信したデータを格納する配列
  static std::vector<std::vector<char>> recv_data_array;

  auto callback = [&](const std_msgs::UInt8MultiArray::ConstPtr &array) {
    // 100発受け取る
    callback_internal(is_received, recv_data_array, 100, array);
  };

  // Destination objectからデータを受け取るSubscriber
  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe<std_msgs::UInt8MultiArray>("/chatter", 1, callback);

  // destination objectを作成
  Injector<DestinationFactory> injector(getDestinationComponent);
  DestinationFactory destinationFactory(injector);
  auto destination =
      destinationFactory("/chatter", udp::endpoint(udp::v4(), 0));

  // startを間違えて数回連続で呼んだ
  destination->Start();
  destination->Start();
  destination->Start();
  destination->Start();
  destination->Start();

  // WebRTC GWの代わりにデータをDestinationに追加する
  std::thread sending_thread([&] { send_udp(destination->Port()); });

  // データの受け渡しが完了するまでspinする
  ros::Rate loop_rate(100);
  while (!ros::isShuttingDown() && !is_received) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  sending_thread.join();
  // stopを間違えて数回連続で呼んだ場合
  destination->Stop();
  destination->Stop();
  destination->Stop();
  destination->Stop();

  // 正しいデータを受け取っている
  for (auto vec : recv_data_array) {
    ASSERT_EQ((int)vec.data()[0] + 1, vec.size());
  }

  // 100発受け取っているはず
  ASSERT_EQ(recv_data_array.size(), 100);
}

// mapの中に格納されていて正常に動作するかのテスト
TEST(TestSuite, destination_insert_map_after_start) {
  // データの受信を完了し、テストを抜けて良くなったらtrueにするフラグ
  bool is_received = false;
  // 受信したデータを格納する配列
  static std::vector<std::vector<char>> recv_data_array;

  auto callback = [&](const std_msgs::UInt8MultiArray::ConstPtr &array) {
    // 100発受け取る
    callback_internal(is_received, recv_data_array, 100, array);
  };

  // Destination objectからデータを受け取るSubscriber
  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe<std_msgs::UInt8MultiArray>("/chatter", 1, callback);

  // destination objectを作成
  Injector<DestinationFactory> injector(getDestinationComponent);
  DestinationFactory destinationFactory(injector);
  auto destination =
      destinationFactory("/chatter", udp::endpoint(udp::v4(), 0));
  // destination objectをmapで管理
  std::unordered_map<std::string, std::unique_ptr<Destination>> map;
  destination->Start();
  map.emplace("hoge", std::move(destination));

  // WebRTC GWの代わりにデータをDestinationに追加する
  std::thread sending_thread([&] { send_udp(map.at("hoge")->Port()); });

  // データの受け渡しが完了するまでspinする
  ros::Rate loop_rate(100);
  while (!ros::isShuttingDown() && !is_received) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  sending_thread.join();
  map.at("hoge")->Stop();

  // 正しいデータを受け取っている
  for (auto vec : recv_data_array) {
    ASSERT_EQ((int)vec.data()[0] + 1, vec.size());
  }

  // 100発受け取っているはず
  ASSERT_EQ(recv_data_array.size(), 100);
}

// mapの中に格納されていて正常に動作するかのテスト
TEST(TestSuite, destination_in_map) {
  // データの受信を完了し、テストを抜けて良くなったらtrueにするフラグ
  bool is_received = false;
  // 受信したデータを格納する配列
  static std::vector<std::vector<char>> recv_data_array;

  auto callback = [&](const std_msgs::UInt8MultiArray::ConstPtr &array) {
    // 100発受け取る
    callback_internal(is_received, recv_data_array, 100, array);
  };

  // Destination objectからデータを受け取るSubscriber
  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe<std_msgs::UInt8MultiArray>("/chatter", 1, callback);

  // destination objectを作成
  Injector<DestinationFactory> injector(getDestinationComponent);
  DestinationFactory destinationFactory(injector);
  auto destination =
      destinationFactory("/chatter", udp::endpoint(udp::v4(), 0));
  // destination objectをmapで管理
  std::unordered_map<std::string, std::unique_ptr<Destination>> map;
  map.emplace("hoge", std::move(destination));

  map.at("hoge")->Start();

  // WebRTC GWの代わりにデータをDestinationに追加する
  std::thread sending_thread([&] { send_udp(map.at("hoge")->Port()); });

  // データの受け渡しが完了するまでspinする
  ros::Rate loop_rate(100);
  while (!ros::isShuttingDown() && !is_received) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  sending_thread.join();
  map.at("hoge")->Stop();

  // 正しいデータを受け取っている
  for (auto vec : recv_data_array) {
    ASSERT_EQ((int)vec.data()[0] + 1, vec.size());
  }

  // 100発受け取っているはず
  ASSERT_EQ(recv_data_array.size(), 100);
}

// startされた後にmapの中に格納されて正常に動作するかのテスト
TEST(TestSuite, destination_in_map_after_start) {
  // データの受信を完了し、テストを抜けて良くなったらtrueにするフラグ
  bool is_received = false;
  // 受信したデータを格納する配列
  static std::vector<std::vector<char>> recv_data_array;

  auto callback = [&](const std_msgs::UInt8MultiArray::ConstPtr &array) {
    // 100発受け取る
    callback_internal(is_received, recv_data_array, 100, array);
  };

  // Destination objectからデータを受け取るSubscriber
  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe<std_msgs::UInt8MultiArray>("/chatter", 1, callback);

  // destination objectを作成
  Injector<DestinationFactory> injector(getDestinationComponent);
  DestinationFactory destinationFactory(injector);
  auto destination =
      destinationFactory("/chatter", udp::endpoint(udp::v4(), 0));
  destination->Start();
  // destination objectをmapで管理
  std::unordered_map<std::string, std::unique_ptr<Destination>> map;
  map.emplace("hoge", std::move(destination));

  // WebRTC GWの代わりにデータをDestinationに追加する
  std::thread sending_thread([&] { send_udp(map.at("hoge")->Port()); });

  // データの受け渡しが完了するまでspinする
  ros::Rate loop_rate(100);
  while (!ros::isShuttingDown() && !is_received) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  sending_thread.join();
  map.at("hoge")->Stop();

  // 正しいデータを受け取っている
  for (auto vec : recv_data_array) {
    ASSERT_EQ((int)vec.data()[0] + 1, vec.size());
  }

  // 100発受け取っているはず
  ASSERT_EQ(recv_data_array.size(), 100);
}

// データの送信中にエンドユーザ側のROS Subscriberが居なくなった場合
TEST(TestSuite, destination_early_exit) {
  // データの受信を完了し、テストを抜けて良くなったらtrueにするフラグ
  bool is_received = false;
  // 受信したデータを格納する配列
  static std::vector<std::vector<char>> recv_data_array;

  auto callback = [&](const std_msgs::UInt8MultiArray::ConstPtr &array) {
    // 3発だけ受け取る
    callback_internal(is_received, recv_data_array, 3, array);
  };

  // Destination objectからデータを受け取るSubscriber
  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe<std_msgs::UInt8MultiArray>("/chatter", 1, callback);

  // destination objectを作成
  Injector<DestinationFactory> injector(getDestinationComponent);
  DestinationFactory destinationFactory(injector);
  auto destination =
      destinationFactory("/chatter", udp::endpoint(udp::v4(), 0));

  destination->Start();

  // WebRTC GWの代わりにデータをDestinationに追加する
  std::thread sending_thread([&] { send_udp(destination->Port()); });

  // データの受け渡しが完了するまでspinする
  ros::Rate loop_rate(100);
  while (!ros::isShuttingDown() && !is_received) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // 早々にsubscriberが終了してしまった場合
  sub.shutdown();

  sending_thread.join();
  destination->Stop();

  // 正しいデータを受け取っている
  for (auto vec : recv_data_array) {
    ASSERT_EQ((int)vec.data()[0] + 1, vec.size());
  }

  // 3発の時点で終了した
  ASSERT_EQ(recv_data_array.size(), 3);
}

// startメソッド実行前はport番号が取得できない
// 仮で0と返すのを確認する
TEST(TestSuite, destination_get_port_before_bind) {
  Injector<DestinationFactory> injector(getDestinationComponent);
  DestinationFactory destinationFactory(injector);
  auto destination =
      destinationFactory("/chatter", udp::endpoint(udp::v4(), 0));
  ASSERT_EQ(destination->Port(), 0);
}