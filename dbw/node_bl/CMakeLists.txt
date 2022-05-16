cmake_minimum_required(VERSION 3.16.0)

if(IGVC_MODULE_TYPE)
  message(STATUS "** Building module ${IGVC_MODULE_TYPE}")
  add_compile_definitions(IGVC_MODULE_TYPE=${IGVC_MODULE_TYPE})
else()
  message(FATAL_ERROR "** STOPPING: No IGVC module specified. **")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(igvc_bl)
