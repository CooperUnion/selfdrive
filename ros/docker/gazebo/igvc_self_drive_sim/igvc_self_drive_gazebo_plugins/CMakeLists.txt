cmake_minimum_required(VERSION 2.8.3)
project(igvc_self_drive_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  gazebo_ros
  dynamic_reconfigure
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

generate_dynamic_reconfigure_options(
  cfg/GazeboCam.cfg
)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}_cam
    ${PROJECT_NAME}_vehicle
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_vehicle
  src/GemInterfacePlugin.cpp
)
target_link_libraries(${PROJECT_NAME}_vehicle
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_library(${PROJECT_NAME}_cam
  src/GazeboCamControl.cpp
)
add_dependencies(${PROJECT_NAME}_cam
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}_cam
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Gazebo plugins
install(
  TARGETS
    ${PROJECT_NAME}_cam
    ${PROJECT_NAME}_vehicle
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
