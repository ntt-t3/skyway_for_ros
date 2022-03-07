#include "router.h"

#include <signal.h>

namespace {
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }
}  // namespace

extern "C" {
char* call_service(const char* message);
char* receive_events();
void release_string(char* message);
}

void RouterImpl::Start() {
  // 終了処理は全てここで行う
  shutdown_handler = [&](int signal) {
    control_service_->Shutdown();
    events_observe_action_->Shutdown();
    // Todo: Peer Objectの削除とその確認後プログラムのshutdownをここでする
  };
  signal(SIGINT, signal_handler);

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
