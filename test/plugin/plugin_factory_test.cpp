#include "../../src/plugin/plugin_factory.h"

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

#include "../../src/plugin/binary_plugin_router.h"
#include "../../src/plugin/json_plugin_router.h"
#include "../../src/plugin/string_plugin_router.h"

using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

/*
class MockJsonSocket : public Socket {
 private:
  std::shared_ptr<std::function<void(std::vector<uint8_t>)>> callback_;
  int counter = 0;

 public:
  // デフォルトコンストラクタは削除
  MockJsonSocket() = delete;
  INJECT(MockJsonSocket(
      ASSISTED(udp::endpoint) target_socket,
      ASSISTED(std::shared_ptr<std::function<void(std::vector<uint8_t>)>>)
          callback))
      : callback_(callback) {}

  virtual void Start() override {
    std::string first_message =
        "{\"key\": \"value\", \"bool\": true, \"num\": 10}";
    std::vector<uint8_t> first_data(first_message.begin(), first_message.end());
    (*callback_)(first_data);

    std::string second_message =
        "{\"key\": \"value\", \"bool\": false, \"num\": 0}";
    std::vector<uint8_t> second_data(second_message.begin(),
                                     second_message.end());
    (*callback_)(second_data);
  }

  virtual void SendData(std::vector<uint8_t> data) override {
    counter += 1;
    std::string message(data.begin(), data.end());
    if (counter == 1) {
      ASSERT_STREQ(message.c_str(),
                   "{\"key\":\"value\",\"bool\":true,\"num\":10}");
    } else {
      ASSERT_STREQ(message.c_str(),
                   "{\"key\":\"value\",\"bool\":false,\"num\":0}");
    }
  }
};

// Mockを入れるもの
Component<SocketFactory> getMockJsonUdpSourceComponent() {
  return createComponent().bind<Socket, MockJsonSocket>();
}

Component<PluginRouterFactory> getMockJsonPluginRouterComponent() {
  return createComponent()
      .replace(getUdpSocketComponent)
      .with(getMockJsonUdpSourceComponent)
      .install(getJsonPluginRouterComponent);
}
*/

// XmlRpcValueが不正なケース
TEST(TestSuite, json) { ASSERT_FALSE(false); }
