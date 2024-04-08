cmake_minimum_required(VERSION 2.8.3)
project(yostlab_imu)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_compile_options(-std=c++11)


# find_package(catkin REQUIRED COMPONENTS
#              roscpp
#              serial
#              rospy
#              sensor_msgs
#              geometry_msgs)

find_package(ament_cmake REQUIRED)
find_package(roscpp REQUIRED)
find_package(serial REQUIRED)
find_package(rospy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

# catkin_package(
#    INCLUDE_DIRS include
#    CATKIN_DEPENDS serial roscpp rospy sensor_msgs geometry_msgs
# )

ament_package()
ament_export_include_directories(include)
ament_export_dependencies(serial roscpp rospy sensor_msgs geometry_msgs)

# include_directories(
#   INCLUDE_DIRS include 
#   ${BOOST_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )

include_directories(
  INCLUDE_DIRS include  
  ${BOOST_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
)

add_executable(yostlab_driver_node src/YostLabDriver.cpp src/yostlab_driver_node.cpp)
# target_link_libraries(yostlab_driver_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(yostlab_driver_node ${rclcpp_LIBRARIES} ${rmw_implementation_LIBRARIES} ${Boost_LIBRARIES})
