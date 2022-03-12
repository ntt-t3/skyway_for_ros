#ifndef SKYWAY_DESTINATION_STUB_H
#define SKYWAY_DESTINATION_STUB_H

#include <fruit/fruit.h>

#include "../../src/domain/entity.h"

using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class DestinationStub : public Destination {
 private:
  bool flag_ = true;

 public:
  std::function<void(int)> callback_;
  DestinationStub() = delete;
  INJECT(DestinationStub(ASSISTED(std::function<void(int)>) callback))
      : callback_(callback) {}
  ~DestinationStub() { Stop(); }
  virtual void Start() override { callback_(1); }
  virtual void Stop() override {
    if (flag_) callback_(2);
    flag_ = false;
  }
  virtual unsigned short Port() override { return 0; }
};

using DestinationStubFactory =
    std::function<std::unique_ptr<Destination>(std::function<void(int)>)>;

Component<DestinationStubFactory> getDestinationStubComponent();

class SimpleDestinationStub : public Destination {
 public:
  SimpleDestinationStub() = delete;
  INJECT(SimpleDestinationStub(ASSISTED(std::string) topic_name,
                               ASSISTED(udp::endpoint) local_endpoint)) {}
  ~SimpleDestinationStub() { Stop(); }
  virtual void Start() override {}
  virtual void Stop() override {}
  virtual unsigned short Port() override { return 0; }
};

Component<DestinationFactory> getSimpleDestinationComponent();

#endif  // SKYWAY_DESTINATION_STUB_H
