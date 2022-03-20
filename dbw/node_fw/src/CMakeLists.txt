# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/base/*.*
                              ${CMAKE_SOURCE_DIR}/src/io/*.*
                              ${CMAKE_SOURCE_DIR}/src/sys/*.*)
idf_component_register(SRCS ${app_sources})
