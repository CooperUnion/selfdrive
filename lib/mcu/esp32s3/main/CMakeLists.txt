idf_component_register(SRCS "empty.c")

idf_build_get_property(components BUILD_COMPONENTS)

add_prebuilt_library(prebuilt "libprebuilt.a" PRIV_REQUIRES ${components})

target_link_libraries(${COMPONENT_LIB} PRIVATE prebuilt)
