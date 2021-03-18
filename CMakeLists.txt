cmake_minimum_required(VERSION 3.0.2)
project(amtc_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES amtc_utils
   CATKIN_DEPENDS roscpp rospy
   DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/Utils.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


