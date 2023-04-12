cmake_minimum_required(VERSION 3.5)
project(skyway)

include(ExternalProject)

SET(VENDOR_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/vendor)

include_directories(
        include
        ${VENDOR_DIR}/fruit/include
)

SET(FRUIT_LIB_FILE ${VENDOR_DIR}/fruit/src/libfruit.a)
ExternalProject_Add(${PROJECT_NAME}_fruit
        URL https://github.com/google/fruit/archive/refs/tags/v3.7.1.zip
        UPDATE_COMMAND ""
        INSTALL_COMMAND ""
        SOURCE_DIR ${VENDOR_DIR}/fruit
        DOWNLOAD_DIR ${VENDOR_DIR}/fruit
        BINARY_DIR ${VENDOR_DIR}/fruit
        CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=Off
        UPDATE_DISCONNECTED 1
        )

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/SkyWayControl.srv"
        "srv/SkyWayEvents.srv"
        )
ament_export_dependencies(rosidl_default_runtime)

add_executable(${PROJECT_NAME}_node
        src/skyway.cpp
        src/ros2/events_service.cpp
        src/ros2/control_service.cpp
        )
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_dependencies(${PROJECT_NAME}_node
        ${PROJECT_NAME}_fruit)

target_link_libraries(${PROJECT_NAME}_node ${FRUIT_LIB_FILE})

ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        pluginlib
        std_msgs
)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(${PROJECT_NAME}_node
        ${PROJECT_NAME} rosidl_typesupport_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
