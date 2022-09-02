//
// Created by nakakura on 22/09/02.
//

#ifndef SKYWAY_PLUGIN_FACTORY_H
#define SKYWAY_PLUGIN_FACTORY_H

#include <fruit/fruit.h>

using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

class PluginFactory {};

class PluginFactoryImpl : public PluginFactory {
 public:
  INJECT(PluginFactoryImpl()) {}
};

Component<PluginFactory> getPluginFactoryComponent();

#endif  // SKYWAY_PLUGIN_FACTORY_H
