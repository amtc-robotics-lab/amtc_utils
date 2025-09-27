cmake_minimum_required(VERSION 3.16.3...3.20)
project(amtc_utils)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake REQUIRED)

find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(resource_manager_msgs REQUIRED)
find_package(autonomous_machine_interface REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(resource_manager_msgs REQUIRED)
find_package(autonomous_machine_interface REQUIRED)
find_package(reflectcpp REQUIRED)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED
        src/${PROJECT_NAME}/Utils.cpp
        src/${PROJECT_NAME}/HeartbeatResource.cpp
        src/${PROJECT_NAME}/NeroResourceClient.cpp
        src/${PROJECT_NAME}/NeroResource.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    autonomous_machine_interface
    resource_manager_msgs
    reflectcpp
)
target_link_libraries(${PROJECT_NAME} reflectcpp::reflectcpp)



ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        reflectcpp
        resource_manager_msgs
        autonomous_machine_interface
)

add_executable(resource_checker
        src/examples/resource_checker.cpp)
ament_target_dependencies(resource_checker
        rclcpp std_msgs)
target_include_directories(resource_checker
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


add_executable(params_example
        src/examples/params_example.cpp)
ament_target_dependencies(params_example
        rclcpp
        reflectcpp
        std_msgs)
target_link_libraries(params_example reflectcpp::reflectcpp)
target_include_directories(params_example
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories("include")
install(
        DIRECTORY include/
        DESTINATION include/
)
install(TARGETS resource_checker params_example
    DESTINATION lib/${PROJECT_NAME})

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_package()
