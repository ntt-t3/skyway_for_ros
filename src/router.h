#ifndef SKYWAY_ROUTER_H
#define SKYWAY_ROUTER_H

#include <fruit/fruit.h>
#include <ros/ros.h>

#include "domain/entity.h"
#include "ffi.h"
#include "presentation/control_service.h"
#include "presentation/events_service.h"

using fruit::Component;
using fruit::Injector;

class Router {
 public:
  virtual ~Router() = default;
  virtual void OnCreatePeer(char* peer_id, char* token) {}
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

  void shutdown(int signal);
  std::string on_control_message(std::string);
  std::string on_event_request();

 public:
  RouterImpl() = delete;
  INJECT(RouterImpl(ControlServiceFactory control_service_factory,
                    EventsServiceFactory event_service_factory,
                    SourceFactory source_factory,
                    DestinationFactory destination_factory,
                    std::shared_ptr<DataTopicContainer> data_topic_container));
  ~RouterImpl() {}

  virtual void OnCreatePeer(char* peer_id, char* token) override;
};

Component<Router> getRouterComponent();

#endif  // SKYWAY_ROUTER_H