cmake_minimum_required(VERSION 3.0.2)
project(apf_pathplanner_tutorial)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  potbot_lib
  potbot_msgs
)

generate_dynamic_reconfigure_options(
  cfg/apf_pathplanner_tutorial.cfg
)

catkin_package(
  LIBRARIES potbot_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/apf_pathplanner_tutorial.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_definitions(-std=c++14)