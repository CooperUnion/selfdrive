cmake_minimum_required(VERSION 2.8.3)
project(igvc_self_drive_gazebo)

find_package(catkin REQUIRED)

catkin_package()

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
