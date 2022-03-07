// 本プログラムではfruitでDIを行う。
// getComponentの実装はこのファイルでまとめて行う。
// release buildの歳のみリンクを行い、Testの際はテスト用の実装をリンクする

#include "presentation/control_service.h"
#include "presentation/events_observe_action.h"
#include "router.h"

Component<ControlServiceFactory> getControlServiceComponent() {
  return createComponent().bind<ControlService, ControlServiceImpl>();
}

Component<EventsObserveActionFactory> getEventsObserveActionComponent() {
  return createComponent().bind<EventsObserveAction, EventsObserveActionImpl>();
}

Component<Router> getRouterComponent() {
  return createComponent()
      .bind<Router, RouterImpl>()
      .install(getControlServiceComponent)
      .install(getEventsObserveActionComponent);
}
