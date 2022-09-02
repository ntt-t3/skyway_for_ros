//
// Created by nakakura on 22/08/25.
//

#include "binary_plugin_router.h"

//===== private =====
void BinaryPluginRouter::observe_socket(std::vector<uint8_t> data) {
  for (auto plugin = plugins_.rbegin(); plugin != plugins_.rend(); ++plugin) {
    (*plugin)->execute(data);
  }
}

void BinaryPluginRouter::observe_plugins(std::vector<uint8_t> data) {
  socket_->SendData(data);
}

//===== public =====
BinaryPluginRouter::BinaryPluginRouter(XmlRpc::XmlRpcValue config,
                                       udp::endpoint target_socket,
                                       SocketFactory factory)
    : plugin_loader_("skyway_plugin", "skyway_plugin::SkyWayBinaryPlugin"),
      config_(config),
      target_socket_(target_socket) {
  // Socketからのcallbackを与えてSocketを生成`
  socket_ = factory(
      target_socket_,
      std::make_shared<std::function<void(std::vector<uint8_t>)>>(std::bind(
          &BinaryPluginRouter::observe_socket, this, std::placeholders::_1)));
}

BinaryPluginRouter::~BinaryPluginRouter() {
  if (socket_) socket_->Stop();
}

PluginResult BinaryPluginRouter::TryStart() {
  if (config_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return {.is_success = false, .error_message = "invalid config parameters"};
  }

  // try startにして、errorを返せるようにする
  auto func_ptr =
      std::make_shared<std::function<void(std::vector<uint8_t>)>>(std::bind(
          &BinaryPluginRouter::observe_plugins, this, std::placeholders::_1));

  XmlRpc::XmlRpcValue array = config_[0]["plugins"];
  for (int i = 0; i < array.size(); ++i) {
    try {
      std::string plugin_name =
          static_cast<std::string>(array[i]["plugin_name"]);
      boost::shared_ptr<skyway_plugin::SkyWayBinaryPlugin> plugin =
          plugin_loader_.createInstance(plugin_name);
      plugin->initialize(array[i], func_ptr);
      plugins_.push_back(plugin);
    } catch (pluginlib::PluginlibException &ex) {
      // pluginがopenできなかったらここでreturnする
      std::string plugin_name =
          static_cast<std::string>(array[i]["plugin_name"]);
      std::ostringstream stream;
      stream << "Failed to load " << plugin_name << ex.what();
      return {.is_success = false, .error_message = stream.str()};
    }
  }

  // ここでsocket startするとデータが流れ始める
  socket_->Start();

  return {.is_success = true, .error_message = ""};
}

Component<fruit::Annotated<BinaryAnnotation, PluginRouterFactory>>
getBinaryPluginRouterComponent() {
  return createComponent()
      .bind<fruit::Annotated<BinaryAnnotation, PluginRouter>,
            BinaryPluginRouter>()
      .install(getUdpSocketComponent);
}
