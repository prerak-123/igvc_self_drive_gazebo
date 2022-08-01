cmake_minimum_required(VERSION 2.8.3)
project(igvc_self_drive_gazebo)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs)

add_action_files(
  DIRECTORY action
  FILES stopline.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(
  modified_vel_arr src/modified_vel_arr.cpp
)

  
add_executable(purepirsuit src/purepirsuit.cpp)

target_link_libraries(
  modified_vel_arr
  ${catkin_LIBRARIES}
)

target_link_libraries(
  purepirsuit
  ${catkin_LIBRARIES}
)

add_dependencies(
  modified_vel_arr
  ${igvc_self_drive_gazebo_EXPORTED_TARGETS}
)

add_dependencies(
  purepirsuit
  ${igvc_self_drive_gazebo_EXPORTED_TARGETS}
)

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
