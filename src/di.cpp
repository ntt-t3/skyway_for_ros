// 本プログラムではfruitでDIを行う。
// getComponentの実装はこのファイルでまとめて行う。
// release buildの歳のみリンクを行い、Testの際はテスト用の実装をリンクする

#include "domain/entity.h"
#include "infra/destination_impl.h"
#include "infra/source_impl.h"
#include "presentation/control_service.h"
#include "presentation/events_service.h"
#include "router.h"

Component<SourceFactory> getSourceComponent() {
  return createComponent().bind<Source, DataChannelSourceImpl>();
}

Component<DataTopicContainer> getDataTopicContainerComponent() {
  return fruit::createComponent()
      .bind<DataTopicContainer, DataTopicContainerImpl>()
      .registerConstructor<DataTopicContainerImpl()>();
}

Component<DestinationFactory> getDestinationComponent() {
  return createComponent().bind<Destination, DataChannelDestinationImpl>();
}

Component<ControlServiceFactory> getControlServiceComponent() {
  return createComponent().bind<ControlService, ControlServiceImpl>();
}

Component<EventsServiceFactory> getEventsServiceComponent() {
  return createComponent().bind<EventsService, EventsServiceImpl>();
}

Component<Router> getRouterComponent() {
  return createComponent()
      .bind<Router, RouterImpl>()
      .install(getControlServiceComponent)
      .install(getEventsServiceComponent)
      .install(getSourceComponent)
      .install(getDestinationComponent)
      .install(getDataTopicContainerComponent);
}
