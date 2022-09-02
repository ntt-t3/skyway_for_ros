#include "../../src/plugin/string_plugin_router.h"

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

#include "../../src/socket/udp_socket.h"
#include "std_msgs/UInt8MultiArray.h"

using boost::asio::ip::udp;
using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class MockStringSocket : public Socket {
 private:
  std::shared_ptr<std::function<void(std::vector<uint8_t>)>> callback_;
  int counter = 0;

 public:
  // デフォルトコンストラクタは削除
  MockStringSocket() = delete;
  INJECT(MockStringSocket(
      ASSISTED(udp::endpoint) target_socket,
      ASSISTED(std::shared_ptr<std::function<void(std::vector<uint8_t>)>>)
          callback))
      : callback_(callback) {}

  virtual void Start() override {
    std::string first_message = "first message";
    std::vector<uint8_t> first_data(first_message.begin(), first_message.end());
    (*callback_)(first_data);

    /*
    std::string second_message = "second message";
    vec.clear();
    vec.resize(second_message.length() * sizeof(char));
    memcpy(&vec[0], second_message.c_str(),
           second_message.length() * sizeof(char));
    (*callback_)(vec);

    std::string third_message = "third message";
    vec.clear();
    vec.resize(third_message.length() * sizeof(char));
    memcpy(&vec[0], third_message.c_str(),
           third_message.length() * sizeof(char));
    (*callback_)(vec);
     */
  }

  virtual void SendData(std::vector<uint8_t> data) override {
    std::string message(data.begin(), data.end());

    counter += 1;
    switch (counter) {
      case 1:
        ASSERT_STREQ(message.c_str(), "first message");
        break;
      case 2:
        ASSERT_STREQ(message.c_str(), "second message");
        break;
      case 3:
        ASSERT_STREQ(message.c_str(), "third message");
        break;
    }
  }
};

// Mockを入れるもの
Component<SocketFactory> getMockStringUdpSourceComponent() {
  return createComponent().bind<Socket, MockStringSocket>();
}

Component<PluginRouterFactory> getMockStringPluginRouterComponent() {
  return createComponent()
      .replace(getUdpSocketComponent)
      .with(getMockStringUdpSourceComponent)
      .install(getStringPluginRouterComponent);
}

// XmlRpcValueが不正なケース
TEST(TestSuite, string_plugin_try_start_with_invalid_xml) {
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue config;

  // objectを作成し、受信スレッドを開始
  Injector<PluginRouterFactory> injector(getMockStringPluginRouterComponent);
  PluginRouterFactory pluginRouterFactory(injector);
  // データは送信しないのでportは何でも良い
  auto source = pluginRouterFactory(config, udp::endpoint(udp::v4(), 0));
  auto result = source->TryStart();
  // TryStartに失敗して、config間違いのエラーメッセージを受け取る
  ASSERT_FALSE(result.is_success);
  ASSERT_EQ(result.error_message, "invalid config parameters");
}

// pluginが見つからないケース
TEST(TestSuite, string_plugin_try_start_not_found_plugin) {
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue config;
  nh.getParam("/test_node/invalid_string_config", config);

  // objectを作成し、受信スレッドを開始
  Injector<PluginRouterFactory> injector(getMockStringPluginRouterComponent);
  PluginRouterFactory pluginRouterFactory(injector);
  // データは送信しないのでportは何でも良い
  auto source = pluginRouterFactory(config, udp::endpoint(udp::v4(), 0));
  auto result = source->TryStart();
  // TryStartに失敗して、pluginがない旨のメッセージを受け取る
  ASSERT_FALSE(result.is_success);
  ASSERT_EQ(result.error_message.rfind("Failed to load"), 0);
}

// Loopback Pluginを使うケース
TEST(TestSuite, string_plugin_try_start_with_loopback_plugin) {
  ros::NodeHandle nh;
  XmlRpc::XmlRpcValue config;
  nh.getParam("/test_node/valid_string_config", config);

  // objectを作成し、受信スレッドを開始
  Injector<PluginRouterFactory> injector(getMockStringPluginRouterComponent);
  PluginRouterFactory pluginRouterFactory(injector);
  // データは送信しないのでportは何でも良い
  auto source = pluginRouterFactory(config, udp::endpoint(udp::v4(), 0));
  auto result = source->TryStart();
  // TryStartに成功する
  // データの送信と評価はMockStringSocket内でやっている
  ASSERT_TRUE(result.is_success);
}
