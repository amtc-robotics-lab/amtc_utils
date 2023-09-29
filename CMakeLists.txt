cmake_minimum_required(VERSION 3.16.3...3.20)
project(amtc_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)



add_library(${PROJECT_NAME} SHARED
    src/${PROJECT_NAME}/Utils.cpp

    )

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    )
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp

    )


install(
  DIRECTORY include/
  DESTINATION include
  )

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )


ament_package()