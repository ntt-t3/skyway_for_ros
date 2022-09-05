//
// Created by nakakura on 22/08/25.
//

#include "json_plugin_router.h"

//===== private =====
void JsonPluginRouter::observe_socket(std::vector<uint8_t> data) {
  std::string message(data.begin(), data.end());
  std::shared_ptr<rapidjson::Document> doc(new rapidjson::Document);
  doc->Parse(message.c_str());

  for (auto plugin = plugins_.rbegin(); plugin != plugins_.rend(); ++plugin) {
    (*plugin)->execute(doc);
  }
}

void JsonPluginRouter::observe_plugins(
    std::shared_ptr<rapidjson::Document> doc) {
  StringBuffer buffer;
  Writer<StringBuffer> writer(buffer);
  doc->Accept(writer);
  std::string s(buffer.GetString(), buffer.GetSize());
  std::vector<uint8_t> data(s.begin(), s.end());
  socket_->SendData(data);
}

//===== public =====
JsonPluginRouter::JsonPluginRouter(XmlRpc::XmlRpcValue config,
                                   udp::endpoint target_socket,
                                   SocketFactory factory)
    : plugin_loader_("skyway_plugin", "skyway_plugin::SkyWayJsonPlugin"),
      config_(config),
      target_socket_(target_socket) {
  // Socketからのcallbackを与えてSocketを生成`
  socket_ = factory(
      target_socket_,
      std::make_shared<std::function<void(std::vector<uint8_t>)>>(std::bind(
          &JsonPluginRouter::observe_socket, this, std::placeholders::_1)));
}

JsonPluginRouter::~JsonPluginRouter() {
  if (socket_) socket_->Stop();
}

PluginResult JsonPluginRouter::TryStart() {
  if (config_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return {.is_success = false, .error_message = "invalid config parameters"};
  }

  // try startにして、errorを返せるようにする
  auto func_ptr = std::make_shared<
      std::function<void(std::shared_ptr<rapidjson::Document>)>>(std::bind(
      &JsonPluginRouter::observe_plugins, this, std::placeholders::_1));

  /*
  XmlRpc::XmlRpcValue array = config_[0]["plugins"];
  for (int i = 0; i < array.size(); ++i) {
    try {
      std::string plugin_name =
          static_cast<std::string>(array[i]["plugin_name"]);
      boost::shared_ptr<skyway_plugin::SkyWayJsonPlugin> plugin =
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
   */

  // ここでsocket startするとデータが流れ始める
  socket_->Start();

  return {.is_success = true, .error_message = ""};
}

Component<fruit::Annotated<JsonAnnotation, PluginRouterFactory>>
getJsonPluginRouterComponent() {
  return createComponent()
      .bind<fruit::Annotated<JsonAnnotation, PluginRouter>, JsonPluginRouter>()
      .install(getUdpSocketComponent);
}
