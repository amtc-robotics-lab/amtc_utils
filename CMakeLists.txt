cmake_minimum_required(VERSION 3.16.3...3.20)
project(amtc_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies(REQUIRED rclcpp tf2 std_msgs resource_manager_msgs rclcpp_lifecycle)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(resource_manager_msgs REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

ament_export_include_directories(include)

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/${PROJECT_NAME}/Utils.cpp
        src/${PROJECT_NAME}/NeroResourceClient.cpp
        src/${PROJECT_NAME}/NeroResource.cpp
)

ament_export_dependencies(
        rclcpp
        resource_manager_msgs
)

ament_auto_add_executable(resource_checker
        src/examples/resource_checker.cpp)
ament_target_dependencies(resource_checker
        rclcpp std_msgs)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})


ament_auto_package()

