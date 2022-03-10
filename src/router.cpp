#include "router.h"

#include <signal.h>

namespace {
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }
std::function<void(char*, char*)> create_peer_callback_handler;
}  // namespace

extern "C" {
struct SourceParameters {
  char* source_topic_name;
  char* destination_address;
  unsigned short destination_port;
};

struct DestinationParameters {
  unsigned short source_port;
  char* destination_topic_name;
};

struct TopicParameters {
  char* data_connection_id;
  SourceParameters source_parameters;
  DestinationParameters destination_parameters;
};
}

using void_char_char_func = void (*)(char*, char*);
using void_char_func = void (*)(char*);
using void_void_func = void (*)();
using void_topicparam_func = void (*)(TopicParameters);

extern "C" {
typedef struct {
  void_char_char_func create_peer_callback;
  void_void_func peer_deleted_callback;
  void_topicparam_func create_data_callback;
} callback_function_t;

void setup_service(callback_function_t& functions);
char* call_service(const char* message);
char* receive_events();
void release_string(char* message);
void shutdown_service(const char* peer_id, const char* token);

void create_peer_callback(char* peer_id, char* token) {
  create_peer_callback_handler(peer_id, token);
  release_string(peer_id);
  release_string(token);
}

void peer_deleted_callback() { ros::shutdown(); }

void create_data_callback(TopicParameters parameter) {
  // todo impl
}
}

void RouterImpl::Start() {
  callback_function_t functions{create_peer_callback, peer_deleted_callback,
                                create_data_callback};
  setup_service(functions);

  // 終了処理は全てここで行う
  shutdown_handler = [&](int signal) {
    control_service_->Shutdown();
    events_observe_action_->Shutdown();
    shutdown_service(peer_id_.c_str(), token_.c_str());
  };
  signal(SIGINT, signal_handler);

  create_peer_callback_handler = [&](char* peer_id, char* token) {
    peer_id_ = peer_id;
    token_ = token;
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
  // SkyWayEvents Actionを起動
  events_observe_action_ =
      events_observe_action_factory_("skyway_events", events_lambda);
}
