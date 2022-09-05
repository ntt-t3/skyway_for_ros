#ifndef SKYWAY_ROUTER_H
#define SKYWAY_ROUTER_H

#include <fruit/fruit.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <ros/ros.h>

#include "domain/entity.h"
#include "ffi.h"
#include "plugin/plugin_router_factory.h"
#include "presentation/control_service.h"
#include "presentation/events_service.h"

using namespace rapidjson;
using fruit::Component;
using fruit::Injector;

class Router {
 public:
  virtual ~Router() = default;
  virtual void OnCreatePeer(char* peer_id, char* token) {}
  virtual void OnConnectData(std::string plugin_type,
                             std::string plugin_param) {}
};

class RouterImpl : public Router {
 private:
  std::string peer_id_ = "";
  std::string token_ = "";
  // factories
  ControlServiceFactory control_service_factory_;
  EventsServiceFactory event_service_factory_;
  SourceFactory source_factory_;
  DestinationFactory destination_factory_;
  // ROS Serviceの実体
  std::unique_ptr<ControlService> control_service_;
  std::unique_ptr<EventsService> event_service_;

  std::shared_ptr<IPluginRouterFactory> plugin_router_factory_;

  void shutdown(int signal);
  std::string on_control_message(std::string);
  std::string on_event_request();

 public:
  RouterImpl() = delete;
  INJECT(RouterImpl(
      ControlServiceFactory control_service_factory,
      EventsServiceFactory event_service_factory, SourceFactory source_factory,
      DestinationFactory destination_factory,
      std::shared_ptr<DataTopicContainer> data_topic_container,
      std::shared_ptr<IPluginRouterFactory> plugin_router_factory));
  ~RouterImpl() {}

  virtual void OnCreatePeer(char* peer_id, char* token) override;
  virtual void OnConnectData(std::string, std::string) override;
};

Component<Router> getRouterComponent();

#endif  // SKYWAY_ROUTER_H