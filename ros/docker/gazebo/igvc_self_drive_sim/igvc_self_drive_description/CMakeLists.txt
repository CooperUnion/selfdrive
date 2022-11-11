cmake_minimum_required(VERSION 2.8.3)
project(igvc_self_drive_description)

find_package(catkin REQUIRED)

# Extract meshes folder
execute_process(
  COMMAND ${CMAKE_COMMAND} -E tar xzf meshes.tar.gz
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

catkin_package()

### Install Rules
install(
  DIRECTORY
    meshes
    urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN *.tar.gz EXCLUDE
)
