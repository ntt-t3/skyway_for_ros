#include "router.h"

#include <signal.h>

namespace {
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }
std::function<void(char*, char*)> create_peer_callback_handler;
std::function<void(TopicParameters parameters)> create_data_callback_handler;
std::function<void(char*)> data_connection_close_event_callback_handler;
}  // namespace

extern "C" {
void create_peer_callback(char* peer_id, char* token) {
  create_peer_callback_handler(peer_id, token);
}

void peer_deleted_callback() { ros::shutdown(); }

PluginLoadResult create_data_callback(char* message) {
  ROS_WARN("create data callback");
  return {
      .is_success = true,
      .port = 51111,
      .error_message = ""
  };
  //create_data_callback_handler(parameter);
}

void data_connection_close_event_callback(char* data_connection_id) {
  data_connection_close_event_callback_handler(data_connection_id);
}
}

void RouterImpl::Start() {
  Function functions{create_peer_callback, peer_deleted_callback,
                     create_data_callback,
                     data_connection_close_event_callback};
  register_callbacks(functions);

  // 終了処理は全てここで行う
  shutdown_handler = [&](int signal) {
    control_service_->Shutdown();
    event_service_->Shutdown();
    // shutdown処理の中身はPeer Objectの開放なので、生成前であれば呼ぶ必要がない
    if (peer_id_ != "" && token_ != "") {
      shutdown_service(peer_id_.c_str(), token_.c_str());
    }
    else
      ros::shutdown();
  };
  signal(SIGINT, signal_handler);

  // Peer Objectの生成に成功したら、peer_idとtokenを保持しておく
  // これは終了時に開放するためだけに利用する
  create_peer_callback_handler = [&](char* peer_id, char* token) {
    peer_id_ = peer_id;
    token_ = token;
    release_string(peer_id);
    release_string(token);
  };

  // SourceTopic, Destination Topicの起動を行う
  create_data_callback_handler = [&](TopicParameters parameter) {
    auto source = source_factory_(
        parameter.source_parameters.source_topic_name,
        udp::endpoint(boost::asio::ip::address::from_string(
                          parameter.source_parameters.destination_address),
                      parameter.source_parameters.destination_port));
    auto destination = destination_factory_(
        parameter.destination_parameters.destination_topic_name,
        udp::endpoint(udp::v4(), parameter.destination_parameters.source_port));
    data_topic_container_->CreateData(parameter.data_connection_id,
                                      std::move(source),
                                      std::move(destination));
    release_string(parameter.source_parameters.destination_address);
    release_string(parameter.source_parameters.source_topic_name);
    release_string(parameter.destination_parameters.destination_topic_name);
    release_string(parameter.data_connection_id);
  };

  // Peer Objectの生成に成功したら、peer_idとtokenを保持しておく
  // これは終了時に開放するためだけに利用する
  data_connection_close_event_callback_handler = [&](char* data_connection_id) {
    data_topic_container_->DeleteData(data_connection_id);
    release_string(data_connection_id);
  };

  // SkyWayControl ServiceをClientが利用した際に呼ばれるコールバック
  std::function<std::string(std::string)> control_lambda =
      [&](std::string request) {
        // これ以降の処理はcallbackを除き全てRust側で実装する
        char* message = call_service(request.c_str());
        // Rust側でCString.into_raw()しているので、開放が必要
        std::string response = message;
        release_string(message);
        return response;
      };
  // SkyWayControl Serviceの起動
  control_service_ = control_service_factory_("skyway_control", control_lambda);

  // SkyWayEvents Actionが呼ばれた際に実際に処理を行うコールバック
  std::function<std::string(void)> events_lambda = [&]() {
    // これ以降の処理はcallbackを除き全てRust側で実装する
    char* message = receive_events();
    // Rust側でCString.into_raw()しているので、開放が必要
    std::string event = message;
    release_string(message);
    return event;
  };
  // SkyWayEvents Serviceを起動
  event_service_ = event_service_factory_("skyway_events", events_lambda);
}
