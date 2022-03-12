#include "destination_stub.h"

Component<DestinationStubFactory> getDestinationStubComponent() {
  return createComponent().bind<Destination, DestinationStub>();
}

Component<DestinationFactory> getSimpleDestinationComponent() {
  return createComponent().bind<Destination, SimpleDestinationStub>();
}
