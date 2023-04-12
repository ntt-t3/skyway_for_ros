#include <skyway/skyway_plugin.hpp>
#include <cmath>

namespace json_plugin
{
    class Square : public skyway_plugin::SkyWayPlugin
    {
    public:
        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return side_length_ * side_length_;
        }

    protected:
        double side_length_;
    };

    class Triangle : public skyway_plugin::SkyWayPlugin
    {
    public:
        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return 0.5 * side_length_ * getHeight();
        }

        double getHeight()
        {
            return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
        }

    protected:
        double side_length_;
    };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(json_plugin::Square, skyway_plugin::SkyWayPlugin)
PLUGINLIB_EXPORT_CLASS(json_plugin::Triangle, skyway_plugin::SkyWayPlugin)
