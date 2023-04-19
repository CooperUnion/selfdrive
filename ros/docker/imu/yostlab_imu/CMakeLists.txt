cmake_minimum_required(VERSION 2.8.3)
project(yostlab_imu)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
             roscpp
             serial
             rospy
             sensor_msgs
             geometry_msgs)


find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS serial roscpp rospy sensor_msgs geometry_msgs
)

include_directories(
  INCLUDE_DIRS include 
  ${BOOST_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_executable(yostlab_driver_node src/YostLabDriver.cpp src/yostlab_driver_node.cpp)
target_link_libraries(yostlab_driver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
