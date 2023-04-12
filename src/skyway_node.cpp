#include <pluginlib/class_loader.hpp>
#include <skyway/skyway_plugin.hpp>

int main(int argc, char** argv)
{
    // To avoid unused parameter warnings
    (void) argc;
    (void) argv;

    pluginlib::ClassLoader<skyway_plugin::SkyWayPlugin> poly_loader("skyway", "skyway_plugin::SkyWayPlugin");

    try
    {
        std::shared_ptr<skyway_plugin::SkyWayPlugin> triangle = poly_loader.createSharedInstance("json_plugin::Triangle");
        triangle->initialize(10.0);

        std::shared_ptr<skyway_plugin::SkyWayPlugin> square = poly_loader.createSharedInstance("json_plugin::Square");
        square->initialize(10.0);

        printf("Triangle area: %.2f\n", triangle->area());
        printf("Square area: %.2f\n", square->area());
    }
    catch(pluginlib::PluginlibException& ex)
    {
        printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }

    return 0;
}
