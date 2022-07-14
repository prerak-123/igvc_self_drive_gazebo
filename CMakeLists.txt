cmake_minimum_required(VERSION 2.8.3)
project(igvc_self_drive_gazebo)

find_package(catkin REQUIRED
roscpp
rospy
std_msgs
message_generation
)

add_message_files(
  FILES
  CV.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

install(
  DIRECTORY
    launch
    models
    worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN *.tar.gz EXCLUDE
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
