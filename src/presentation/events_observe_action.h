// SkyWay WebRTC Gateway Caller(以下Caller)を利用して、
// WebRTC Gateway内で発生したイベントを取得するためのクラス
// Callerから帰ってきたイベントを通知するためのROS Actionを提供する
//
// クライアントから要求がくると、まず終了処理中ではないか確認を行う。
// 終了処理中でなければ、オブザーバに対して通知を行う。
// 通知を受けたオブザーバーは、Callerからイベントを取得して戻り値として返す。
// 戻り値を受け取ったらActionとして返す。

#ifndef SKYWAY_EVENTS_NOTIFIER_H
#define SKYWAY_EVENTS_NOTIFIER_H

#include <actionlib/server/simple_action_server.h>
#include <fruit/fruit.h>
#include <ros/ros.h>
#include <skyway/SkyWayEventsAction.h>

#include <functional>
#include <string>

using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class EventsObserveAction {
 public:
  virtual ~EventsObserveAction() = default;
  virtual void Shutdown() {}
};

class EventsObserveActionImpl : public EventsObserveAction {
 private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<skyway::SkyWayEventsAction> as_;
  std::string action_name_;
  std::function<std::string(void)> observer_;
  skyway::SkyWayEventsFeedback feedback_;
  skyway::SkyWayEventsResult result_;
  bool is_running_ = true;

  // Actionに接続すると、Callerからのイベントを取得に行き、
  // 帰ってきたイベントをエンドユーザプログラムに返す
  void callback(const skyway::SkyWayEventsGoalConstPtr &goal);

 public:
  // コンストラクタではAction名とReceiver Objectを受け取る
  INJECT(EventsObserveActionImpl(ASSISTED(std::string) name,
                                 ASSISTED(std::function<std::string(void)>)
                                     observer));
  ~EventsObserveActionImpl() {}
  virtual void Shutdown() override {
    // 明示的にActionServerをshutdownはしない。
    // これ以上処理は受け付けないようにして、ros::shutdownに任せる
    is_running_ = false;
  }
};

using EventsObserveActionFactory =
    std::function<std::unique_ptr<EventsObserveAction>(
        std::string, std::function<std::string(void)>)>;

fruit::Component<EventsObserveActionFactory> getEventsObserveActionComponent();

#endif
