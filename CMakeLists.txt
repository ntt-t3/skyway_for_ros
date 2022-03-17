cmake_minimum_required(VERSION 3.0.2)
project(skyway)
add_compile_options(-std=c++14)

include(ExternalProject)

SET(VENDOR_DIR ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/vendor)

SET(SKYWAY_LIB_FILE ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/rust_module/target/release/libskyway.a)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        std_msgs
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
        )

aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/presentation PRESENTATION)
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/domain DOMAIN)
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/infra INFRA)

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/router.cpp
        src/di.cpp
        ${PRESENTATION}
        ${DOMAIN}
        ${INFRA}
        )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_fruit)

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
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/test/domain DOMAIN_TESTS)
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/test/infra INFRA_TESTS)
aux_source_directory(${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/test/stub STUB)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}_test_node test/unit_test.test
            ${TESTS}
            ${DOMAIN_TESTS}
            ${INFRA_TESTS}
            ${STUB}
            ${DOMAIN}
            ${INFRA}
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