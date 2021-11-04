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
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES traj_gen
#  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

add_executable(${PROJECT_NAME}_node src/traj_gen_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
