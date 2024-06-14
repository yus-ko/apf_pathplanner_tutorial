cmake_minimum_required(VERSION 3.0.2)
project(apf_pathplanner_tutorial)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  potbot_lib
  potbot_msgs
  dynamic_reconfigure
  interactive_markers
)

generate_dynamic_reconfigure_options(
  cfg/APFPathPlannerTutorial.cfg
)

catkin_package(
  LIBRARIES potbot_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/apf_pathplanner_tutorial.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(interactive_marker_node src/interactive_marker_node.cpp)
target_link_libraries(interactive_marker_node ${catkin_LIBRARIES})

add_definitions(-std=c++14)