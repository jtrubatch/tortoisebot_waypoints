cmake_minimum_required(VERSION 3.0.2)
project(tortoisebot_waypoints)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  genmsg
  actionlib_msgs
  actionlib
  geometry_msgs
)

# catkin_python_setup()
add_action_files(
  FILES
  WaypointAction.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)
catkin_package(
  LIBRARIES tortoisebot_waypoints
  CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime actionlib_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/tortoisebot_action_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
