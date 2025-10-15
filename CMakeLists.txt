cmake_minimum_required(VERSION 3.16.3...3.20)
project(amtc_utils)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Debug)

option(ENABLE_PARAMS_HELPER "use to disable param support and dependency on reflect-cpp" ON)

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


if (ENABLE_PARAMS_HELPER)
    find_package(reflectcpp REQUIRED)
endif()


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

ament_export_include_directories(include)

add_library(${PROJECT_NAME} SHARED
        src/${PROJECT_NAME}/Utils.cpp
        src/${PROJECT_NAME}/HeartbeatResource.cpp
        src/${PROJECT_NAME}/NeroResourceClient.cpp
        src/${PROJECT_NAME}/NeroResource.cpp
        src/${PROJECT_NAME}/params_helper.cpp
        src/${PROJECT_NAME}/change_parameters_interface.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    autonomous_machine_interface
    resource_manager_msgs
)

if (ENABLE_PARAMS_HELPER)
    ament_target_dependencies(${PROJECT_NAME}
       reflectcpp
   )
    target_link_libraries(${PROJECT_NAME} reflectcpp::reflectcpp)
endif()




ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
        reflectcpp
        resource_manager_msgs
        autonomous_machine_interface
)
if (ENABLE_PARAMS_HELPER)
    ament_export_dependencies(
                reflectcpp
        )
endif()

add_executable(resource_checker
        src/examples/resource_checker.cpp)
ament_target_dependencies(resource_checker
        rclcpp std_msgs)
target_include_directories(resource_checker
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)


if (ENABLE_PARAMS_HELPER)
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

    add_executable(params_changer_example
                src/examples/params_changer_example.cpp)
    ament_target_dependencies(params_changer_example
                rclcpp
                reflectcpp
                std_msgs)
        target_link_libraries(params_changer_example reflectcpp::reflectcpp ${PROJECT_NAME})
    target_include_directories(params_changer_example
                PUBLIC
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<INSTALL_INTERFACE:include>)
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories("include")
install(
        DIRECTORY include/
        DESTINATION include/
)
if (ENABLE_PARAMS_HELPER)
    install(TARGETS params_example params_changer_example
            DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS resource_checker
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
