cmake_minimum_required(VERSION 3.0.2)
project(traj_gen)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  mav_trajectory_generation
  mav_trajectory_generation_ros
  roslaunch
  dynamic_reconfigure
  geometric_controller
  controller_msgs
  message_generation
)

add_message_files(
  FILES
  FlatTarget.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES traj_gen
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp rospy sensor_msgs std_msgs mav_trajectory_generation mav_trajectory_generation_ros dynamic_reconfigure geometric_controller controller_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

add_executable(traj_gen_node src/traj_gen_node.cpp)
add_executable(geometric_controller src/geometric_controller.cpp)
add_executable(command_point_node src/command_point_node.cpp)

target_link_libraries(traj_gen_node
  ${catkin_LIBRARIES}
)
target_link_libraries(geometric_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(command_point_node
  ${catkin_LIBRARIES}
)