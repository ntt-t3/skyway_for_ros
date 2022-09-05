//
// Created by nakakura on 22/08/25.
//

#include "string_plugin_router.h"

//===== private =====
void StringPluginRouter::observe_socket(std::vector<uint8_t> data) {
  std::string message(data.begin(), data.end());

  for (auto plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin) {
    (*plugin)->execute(message);
  }
}

void StringPluginRouter::observe_plugins(std::string message) {
  std::vector<uint8_t> data(message.begin(), message.end());
  socket_->SendData(data);
}

//===== public =====
StringPluginRouter::StringPluginRouter(
    std::shared_ptr<rapidjson::Document> config, udp::endpoint target_socket,
    SocketFactory factory)
    : plugin_loader_("skyway_plugin", "skyway_plugin::SkyWayStringPlugin"),
      config_(std::move(config)),
      target_socket_(target_socket) {
  // Socketからのcallbackを与えてSocketを生成`
  socket_ = factory(
      target_socket_,
      std::make_shared<std::function<void(std::vector<uint8_t>)>>(std::bind(
          &StringPluginRouter::observe_socket, this, std::placeholders::_1)));
}

StringPluginRouter::~StringPluginRouter() {
  if (socket_) socket_->Stop();
}

// try startにして、errorを返せるようにする
PluginResult StringPluginRouter::TryStart() {
  // plugin情報の配列を与えられていない場合は開始できない
  if (!config_->IsArray()) {
    return {.is_success = false, .error_message = "invalid config parameters"};
  }

  auto callback = std::make_shared<std::function<void(std::string)>>(std::bind(
      &StringPluginRouter::observe_plugins, this, std::placeholders::_1));

  for (rapidjson::Value::ConstValueIterator itr = config_->Begin();
       itr != config_->End(); ++itr) {
    if (!itr->HasMember("plugin_name")) continue;
    std::string plugin_name = (*itr)["plugin_name"].GetString();

    try {
      boost::shared_ptr<skyway_plugin::SkyWayStringPlugin> plugin =
          plugin_loader_.createInstance(plugin_name.c_str());
      std::shared_ptr<rapidjson::Document> parameter(new rapidjson::Document);
      parameter->CopyFrom(*itr, parameter->GetAllocator());
      plugin->initialize(std::move(parameter), callback);
      plugins_.push_back(plugin);
    } catch (pluginlib::PluginlibException &ex) {
      // pluginがopenできなかったらここでreturnする
      std::ostringstream stream;
      stream << "Failed to load " << plugin_name << ex.what();
      return {.is_success = false, .error_message = stream.str()};
    }
  }

  // ここでsocket startするとデータが流れ始める
  socket_->Start();

  return {.is_success = true, .port = socket_->Port(), .error_message = ""};
}

uint16_t StringPluginRouter::Port() { return socket_->Port(); }

Component<fruit::Annotated<StringAnnotation, PluginRouterFactory>>
getStringPluginRouterComponent() {
  return createComponent()
      .bind<fruit::Annotated<StringAnnotation, PluginRouter>,
            StringPluginRouter>()
      .install(getUdpSocketComponent);
}
