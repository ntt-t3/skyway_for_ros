#include <skyway/binary_loopback.h>
#include <skyway/json_loopback.h>
#include <skyway/skyway_plugin.h>
#include <skyway/string_loopback.h>

PLUGINLIB_EXPORT_CLASS(binary_loopback::BinaryLoopback,
                       skyway_plugin::SkyWayBinaryPlugin)
PLUGINLIB_EXPORT_CLASS(json_loopback::JsonLoopback,
                       skyway_plugin::SkyWayJsonPlugin)
PLUGINLIB_EXPORT_CLASS(string_loopback::StringLoopback,
                       skyway_plugin::SkyWayStringPlugin)
