// 本プログラムではfruitでDIを行う。
// getComponentの実装はこのファイルでまとめて行う。
// release buildの歳のみリンクを行い、Testの際はテスト用の実装をリンクする

#include "plugin_router/binary_plugin_router.h"
#include "plugin_router/json_plugin_router.h"
#include "plugin_router/string_plugin_router.h"
#include "presentation/control_service.h"
#include "presentation/events_service.h"
#include "socket/udp_socket.h"

Component<ControlServiceFactory> getControlServiceComponent() {
  return createComponent().bind<ControlService, ControlServiceImpl>();
}

Component<EventsServiceFactory> getEventsServiceComponent() {
  return createComponent().bind<EventsService, EventsServiceImpl>();
}
