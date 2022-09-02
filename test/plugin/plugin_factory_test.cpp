#include "../../src/plugin/plugin_factory.h"

#include <fruit/fruit.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <unordered_map>

#include "../../src/plugin/binary_plugin_router.h"
#include "../../src/plugin/json_plugin_router.h"
#include "../../src/plugin/string_plugin_router.h"

using fruit::Component;
using fruit::createComponent;
using fruit::Injector;

TEST(TestSuite, create) {
  Injector<PluginFactory> injector(getPluginFactoryComponent);
  auto pluginFactory = injector.get<PluginFactory*>();
  ROS_ERROR("%d", pluginFactory->Test());
  pluginFactory->say();

  ASSERT_FALSE(false);
}
