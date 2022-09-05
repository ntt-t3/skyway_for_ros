cmake_minimum_required(VERSION 3.0.2)
project(skyway)
add_compile_options(-std=c++14)

include(ExternalProject)

SET(VENDOR_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/vendor)

SET(SKYWAY_LIB_FILE ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/rust_module/target/release/libskyway.a)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        std_msgs
        pluginlib
        skyway_plugin
        )

add_service_files(
        DIRECTORY srv
        FILES SkyWayControl.srv SkyWayEvents.srv
)

generate_messages(
        DEPENDENCIES std_msgs
)

catkin_package(
        CATKIN_DEPENDS std_msgs message_runtime
)


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${VENDOR_DIR}/fruit/include
        ${VENDOR_DIR}/rapidjson/include
)

###########
## Build ##
###########

# add DI framework
SET(FRUIT_LIB_FILE ${VENDOR_DIR}/fruit/src/libfruit.a)
ExternalProject_Add(${PROJECT_NAME}_fruit
        GIT_REPOSITORY https://github.com/google/fruit.git
        GIT_TAG v3.6.0
        DOWNLOAD_COMMAND git clone --depth 1 https://github.com/google/fruit.git .
        UPDATE_COMMAND ""
        INSTALL_COMMAND ""
        SOURCE_DIR ${VENDOR_DIR}/fruit
        DOWNLOAD_DIR ${VENDOR_DIR}/fruit
        BINARY_DIR ${VENDOR_DIR}/fruit
        CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=Off
        UPDATE_DISCONNECTED 1
        )

# add json library
ExternalProject_Add(${PROJECT_NAME}_rapidjson
        GIT_REPOSITORY https://github.com/Tencent/rapidjson.git
        GIT_TAG v1.1.0
        DOWNLOAD_COMMAND git clone --depth 1 https://github.com/Tencent/rapidjson.git .
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        SOURCE_DIR ${VENDOR_DIR}/rapidjson
        DOWNLOAD_DIR ${VENDOR_DIR}/rapidjson
        BINARY_DIR ${VENDOR_DIR}/rapidjson
        UPDATE_DISCONNECTED 1
        )

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${VENDOR_DIR})

aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/plugin PLUGIN)
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/socket SOCKET)
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/presentation PRESENTATION)
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/infra INFRA)

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/router.cpp
        src/di.cpp
        src/ffi.cpp
        src/ffi_bridge.cpp
        ${SOCKET}
        ${PLUGIN}
        ${PRESENTATION}
        ${INFRA}
        )

add_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_fruit
        ${PROJECT_NAME}_rapidjson)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${SKYWAY_LIB_FILE}
        ${FRUIT_LIB_FILE}
        crypto
        ssl
        pthread
        dl
        rt
        )

aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/test TESTS)
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/test/socket SOCKET_TESTS)
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/test/plugin PLUGIN_TESTS)
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/test/stub STUB)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}_test_node test/unit_test.test
            ${TESTS}
            ${SOCKET_TESTS}
            ${PLUGIN_TESTS}
            ${STUB}
            ${INFRA}
            ${SOCKET}
            ${PLUGIN}
            )
    target_link_libraries(${PROJECT_NAME}_test_node
            ${catkin_LIBRARIES}
            ${FRUIT_LIB_FILE}
            crypto
            ssl
            pthread
            dl
            rt
            )
endif ()