#include "events_observe_action.h"

void EventsObserveActionImpl::callback(
    const skyway::SkyWayEventsGoalConstPtr &goal) {
  // 終了処理中は新規でCallerにイベントを取りに行かず、
  // 終了処理中の通知だけ返す。
  // さらにアクセスが来ないよう、イベントはabortする
  if (!is_running_ || ros::isShuttingDown()) {
    result_.result =
        "{\"is_success\": false, \"error\": \"skyway service is "
        "shutting down\"}";
    as_.setAborted(result_);
    return;
  }

  feedback_.feedback.clear();
  feedback_.feedback = "getting events";
  as_.publishFeedback(feedback_);
  result_.result = observer_();
  as_.setSucceeded(result_);
}

// コンストラクタではAction名とReceiver Objectを受け取る
EventsObserveActionImpl::EventsObserveActionImpl(
    ASSISTED(std::string) name,
    ASSISTED(std::function<std::string(void)>) observer)
    : as_(nh_, name, boost::bind(&EventsObserveActionImpl::callback, this, _1),
          false),
      action_name_(name),
      observer_(observer) {
  as_.start();
}
