#ifndef SKYWAY_ROUTER_H
#define SKYWAY_ROUTER_H

// presentation層を通してEnd User Programが送ったメッセージを
// application層以下のオブジェクトに渡し、処理結果をpresentation層を通して返すという
// 全体の処理の流れを定義するクラス

#include <fruit/fruit.h>

#include "domain/entity.h"
#include "presentation/control_service.h"
#include "presentation/events_observe_action.h"

using fruit::Component;
using fruit::Injector;

class Router {
 public:
  ~Router() = default;
  virtual void Start() {}
};

class RouterImpl : public Router {
 private:
  ControlServiceFactory control_service_factory_;
  EventsObserveActionFactory events_observe_action_factory_;
  SourceFactory source_factory_;
  DestinationFactory destination_factory_;
  std::shared_ptr<DataTopicContainer> data_topic_container_;
  std::unique_ptr<ControlService> control_service_;
  std::unique_ptr<EventsObserveAction> events_observe_action_;
  std::string peer_id_ = "";
  std::string token_ = "";

 public:
  INJECT(RouterImpl(ControlServiceFactory control_service_factory,
                    EventsObserveActionFactory events_observe_action_factory,
                    SourceFactory source_factory,
                    DestinationFactory destination_factory,
                    std::shared_ptr<DataTopicContainer> data_topic_container))
      : control_service_factory_(control_service_factory),
        events_observe_action_factory_(events_observe_action_factory),
        source_factory_(source_factory),
        destination_factory_(destination_factory),
        data_topic_container_(data_topic_container) {}

  // プログラム全体で処理を開始する
  // presentation層のTopic, Actionもこのタイミングで起動する
  virtual void Start();
};

fruit::Component<Router> getRouterComponent();

// ========== Parameters for FFI ==========

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

using void_char_char_func = void (*)(char*, char*);
using void_char_func = void (*)(char*);
using void_void_func = void (*)();
using void_topicparam_func = void (*)(TopicParameters);

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
void create_peer_callback(char* peer_id, char* token);
void peer_deleted_callback();
void create_data_callback(TopicParameters parameter);
}

#endif  // SKYWAY_ROUTER_H
