cmake_minimum_required(VERSION 3.5)
project(skyway)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)

# Plugin Definition
add_library(skyway_plugin SHARED src/plugin_ros2/skyway_plugin.cpp)
target_include_directories(skyway_plugin PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(skyway_plugin pluginlib)
pluginlib_export_plugin_description_file(skyway plugins.xml)
target_compile_definitions(skyway_plugin PRIVATE "POLYGON_PLUGINS_BUILDING_LIBRARY")
install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS skyway_plugin
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Executable Definition
add_executable(skyway_node src/skyway_node.cpp)
target_include_directories(skyway_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(skyway_node pluginlib)
add_dependencies(skyway_node skyway_plugin)
install(
        DIRECTORY include/
        DESTINATION include
)
install(TARGETS skyway_node
        DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export targets
ament_export_include_directories(include)
ament_export_libraries(skyway_plugin)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
