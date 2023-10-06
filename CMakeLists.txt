cmake_minimum_required(VERSION 3.16.3...3.20)
project(amtc_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)



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



add_executable(resource_checker
    src/examples/resource_checker.cpp)
ament_target_dependencies(resource_checker          
  rclcpp std_msgs)
target_include_directories(resource_checker
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
  
target_link_libraries(  resource_checker ${PROJECT_NAME} )
  
install( TARGETS resource_checker
DESTINATION lib/${PROJECT_NAME} )


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
