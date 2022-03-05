cmake_minimum_required(VERSION 3.0.2)
project(skyway)
add_compile_options(-std=c++14)

include(ExternalProject)

find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        message_generation
        roscpp
        std_msgs
        )

add_action_files(
        DIRECTORY action
        FILES SkyWayEvents.action
)

add_service_files(
        DIRECTORY srv
        FILES SkyWayControl.srv
)

generate_messages(
        DEPENDENCIES
        actionlib_msgs std_msgs
)

catkin_package(
        CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime
)


include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        crypto
        ssl
        pthread
        dl
        rt
        )
