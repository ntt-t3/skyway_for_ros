//
// Created by nakakura on 4/12/23.
//

#ifndef SKYWAY_SKYAWY_PLUGIN_HPP
#define SKYWAY_SKYAWY_PLUGIN_HPP

namespace skyway_plugin
{
    class SkyWayPlugin
    {
    public:
        virtual void initialize(double side_length) = 0;
        virtual double area() = 0;
        virtual ~SkyWayPlugin(){}

    protected:
        SkyWayPlugin(){}
    };
}  // namespace skyway_plugin

#endif //SKYWAY_SKYAWY_PLUGIN_HPP
