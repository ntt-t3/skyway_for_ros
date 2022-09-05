#include "router.h"

namespace {
std::function<void(int)> shutdown_handler;
void signal_handler(int signal) { shutdown_handler(signal); }
}  // namespace

RouterImpl::RouterImpl(ControlServiceFactory control_service_factory,
                       EventsServiceFactory event_service_factory,
                       SourceFactory source_factory,
                       DestinationFactory destination_factory,
                       std::shared_ptr<DataTopicContainer> data_topic_container)
    : control_service_factory_(control_service_factory),
      event_service_factory_(event_service_factory),
      source_factory_(source_factory),
      destination_factory_(destination_factory) {
  shutdown_handler =
      std::bind(&RouterImpl::shutdown, this, std::placeholders::_1);
  // 終了時にROS-nodeが落ちる前に開放処理をしなければならないので、SIGINTをhookする
  signal(SIGINT, signal_handler);

  ROS_DEBUG("start /skyway_control");
  // SkyWayControl Serviceの起動
  control_service_ = control_service_factory_(
      "skyway_control",
      std::bind(&RouterImpl::on_control_message, this, std::placeholders::_1));
  ROS_DEBUG("start /skyway_events");
  // SkyWayEvent Serviceの起動
  event_service_ = event_service_factory_(
      "skyway_events", std::bind(&RouterImpl::on_event_request, this));
}

std::string RouterImpl::on_control_message(std::string request) {
  char* message = call_service(request.c_str());
  // Rust側でCString.into_raw()しているので、開放が必要
  std::string response = message;
  release_string(message);
  return response;
}

std::string RouterImpl::on_event_request() {
  // これ以降の処理はcallbackを除き全てRust側で実装する
  char* message = receive_events();
  // Rust側でCString.into_raw()しているので、開放が必要
  std::string event = message;
  release_string(message);
  return event;
}

void RouterImpl::shutdown(int signal) {
  // 終了処理は全てここで行う
  // control_service_->Shutdown();
  // event_service_->Shutdown();
  // shutdown処理の中身はPeer Objectの開放なので、生成前であれば呼ぶ必要がない
  if (peer_id_ != "" && token_ != "") {
    // shutdown_service(peer_id_.c_str(), token_.c_str());
  } else
    ros::shutdown();
}

void RouterImpl::OnCreatePeer(char* peer_id, char* token) {
  // Peer Objectの生成に成功したら、peer_idとtokenを保持しておく
  // これは終了時に開放するためだけに利用する
  peer_id_ = peer_id;
  token_ = token;
}

void RouterImpl::OnConnectData(std::string plugin_type,
                               std::string plugin_param) {
  ROS_WARN("type %s", plugin_type.c_str());
  ROS_WARN("type %s", plugin_param.c_str());
  /*
   TODO
  rapidjson::Document doc;
  doc.Parse(json_str.c_str());

  if (!doc.HasMember("type")) return;

  std::string type = doc["type"].GetString();
  ROS_ERROR("type %s", type.c_str());

  if (!doc["plugins"].IsArray()) return;
   */

  /*
  for (int i = 0; i < doc["plugins"].Size(); i++) {
  }
   */

  /*
  const char* const json = "{\"serial\": 7}";

  xmlrpc_value* const valP = xmlrpc_parse_json(&env, json);

  xmlrpc_value* serialP;
  int serial;

  xmlrpc_struct_find_value(&env, valP, "serial", &serialP);

  xmlrpc_read_int(&env, serialP, &serial);
  assert(serial == 7);
   */

  /*
  for (SizeType i = 0; i < a.Size(); i++)  // Uses SizeType instead of size_t
    printf("a[%d] = %d\n", i, a[i].GetInt());
    */
}

Component<Router> getRouterComponent() {
  return fruit::createComponent()
      .bind<Router, RouterImpl>()
      .install(getControlServiceComponent)
      .install(getEventsServiceComponent)
      .install(getSourceComponent)
      .install(getDestinationComponent)
      .install(getDataTopicContainerComponent);
}
