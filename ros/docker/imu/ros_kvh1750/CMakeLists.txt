cmake_minimum_required(VERSION 2.8.3)
project(kvh1750)

# check c++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)

if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wshadow -fPIC")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             sensor_msgs
             serial
             tf
             pluginlib
            )

catkin_package(
INCLUDE_DIRS include
#  LIBRARIES
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(kvh1750
            src/iomodule.cpp
            src/types.cpp
            src/imu.cpp
            src/tov_file.cpp
            src/bit_message.cpp
           )

catkin_add_gtest(kvh1750-test test/utest.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(kvh1750-test kvh1750 ${catkin_LIBRARIES})
endif()

##
add_executable(kvh_1750_imu src/node.cpp)
target_link_libraries(kvh_1750_imu kvh1750 ${catkin_LIBRARIES})

##
install(TARGETS kvh_1750_imu kvh1750
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY config
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/kvh1750
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
