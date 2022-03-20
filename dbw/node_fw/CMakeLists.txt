cmake_minimum_required(VERSION 3.16.0)

if(IGVC_MODULE_TYPE)
  message(STATUS "** Building module ${IGVC_MODULE_TYPE}")
  list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/mod/${IGVC_MODULE_TYPE}")
else()
  message(FATAL_ERROR "** STOPPING: No IGVC module specified. **")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node_firmware2)
