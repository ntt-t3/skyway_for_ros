#include "../../src/plugin/binary_plugin_router.h"

#include <fruit/fruit.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <unordered_map>

#include "../../src/socket/socket.h"
#include "std_msgs/UInt8MultiArray.h"

using boost::asio::ip::udp;
using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class MockBinarySocket : public Socket {
 private:
  std::shared_ptr<std::function<void(std::vector<uint8_t>)>> callback_;

 public:
  // デフォルトコンストラクタは削除
  MockBinarySocket() = delete;
  INJECT(MockBinarySocket(
      ASSISTED(udp::endpoint) target_socket,
      ASSISTED(std::shared_ptr<std::function<void(std::vector<uint8_t>)>>)
          callback))
      : callback_(callback) {}

  virtual void Start() override {
    std::vector<uint8_t> vec{0, 1};
    (*callback_)(vec);

    vec.push_back(2);
    (*callback_)(vec);

    vec.push_back(3);
    (*callback_)(vec);
  }

  virtual void SendData(std::vector<uint8_t> data) override {
    for (int i = 0; i < data.size(); i++) {
      ASSERT_EQ(data[i], i);
    }
  }
};

// Mockを入れるもの
Component<SocketFactory> getMockBinarySourceComponent() {
  return createComponent().bind<Socket, MockBinarySocket>();
}

Component<PluginRouterFactory> getMockBinaryPluginRouterComponent() {
  return createComponent()
      .replace(getUdpSocketComponent)
      .with(getMockBinarySourceComponent)
      .install(getBinaryPluginRouterComponent);
}

// XmlRpcValueが不正なケース
TEST(TestSuite, binary_plugin_try_start_with_invalid_xml) {
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue config;

  // objectを作成し、受信スレッドを開始
  Injector<PluginRouterFactory> injector(getMockBinaryPluginRouterComponent);
  PluginRouterFactory pluginRouterFactory(injector);
  // データは送信しないのでportは何でも良い
  auto source = pluginRouterFactory(config, udp::endpoint(udp::v4(), 0));
  auto result = source->TryStart();
  // TryStartに失敗して、config間違いのエラーメッセージを受け取る
  ASSERT_FALSE(result.is_success);
  ASSERT_EQ(result.error_message, "invalid config parameters");
}

// pluginが見つからないケース
TEST(TestSuite, binary_plugin_try_start_not_found_plugin) {
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue config;
  nh.getParam("/test_node/invalid_binary_config", config);

  // objectを作成し、受信スレッドを開始
  Injector<PluginRouterFactory> injector(getMockBinaryPluginRouterComponent);
  PluginRouterFactory pluginRouterFactory(injector);
  // データは送信しないのでportは何でも良い
  auto source = pluginRouterFactory(config, udp::endpoint(udp::v4(), 0));
  auto result = source->TryStart();
  // TryStartに失敗して、pluginがない旨のメッセージを受け取る
  ASSERT_FALSE(result.is_success);
  ASSERT_EQ(result.error_message.rfind("Failed to load"), 0);
}

// Loopback Pluginを使うケース
TEST(TestSuite, binary_plugin_try_start_with_loopback_plugin) {
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue config;
  nh.getParam("/test_node/valid_binary_config", config);

  // objectを作成し、受信スレッドを開始
  Injector<PluginRouterFactory> injector(getMockBinaryPluginRouterComponent);
  PluginRouterFactory pluginRouterFactory(injector);
  // データは送信しないのでportは何でも良い
  auto source = pluginRouterFactory(config, udp::endpoint(udp::v4(), 0));
  auto result = source->TryStart();
  // TryStartに成功する
  // データの送信と評価はMockBinarySocket内でやっている
  ASSERT_TRUE(result.is_success);
}
