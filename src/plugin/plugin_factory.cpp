//
// Created by nakakura on 22/09/02.
//

#include "plugin_factory.h"

Component<PluginFactory> getPluginFactoryComponent() {
  return createComponent().bind<PluginFactory, PluginFactoryImpl>();
}
