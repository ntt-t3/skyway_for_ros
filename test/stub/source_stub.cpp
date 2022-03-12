#include "source_stub.h"

Component<SourceStubFactory> getSourceStubComponent() {
  return createComponent().bind<Source, SourceStub>();
}

Component<SourceFactory> getSimpleSourceComponent() {
  return createComponent().bind<Source, SimpleSourceStub>();
}
