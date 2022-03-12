#ifndef SKYWAY_SOURCE_STUB_H
#define SKYWAY_SOURCE_STUB_H

#include <fruit/fruit.h>

#include "../../src/domain/entity.h"

using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class SourceStub : public Source {
 private:
  bool flag_ = true;

 public:
  std::function<void(int)> callback_;
  SourceStub() = delete;
  INJECT(SourceStub(ASSISTED(std::function<void(int)>) callback))
      : callback_(callback) {}
  ~SourceStub() { Stop(); }
  virtual void Start() override { callback_(1); }
  virtual void Stop() override {
    if (flag_) callback_(2);
    flag_ = false;
  }
};

using SourceStubFactory =
    std::function<std::unique_ptr<Source>(std::function<void(int)>)>;

Component<SourceStubFactory> getSourceStubComponent();

class SimpleSourceStub : public Source {
 public:
  SimpleSourceStub() = delete;
  INJECT(SimpleSourceStub(ASSISTED(std::string) topic_name,
                          ASSISTED(udp::endpoint) remote_endpoint)) {}
  ~SimpleSourceStub() { Stop(); }
  virtual void Start() override {}
  virtual void Stop() override {}
};

Component<SourceFactory> getSimpleSourceComponent();

#endif  // SKYWAY_SOURCE_STUB_H
