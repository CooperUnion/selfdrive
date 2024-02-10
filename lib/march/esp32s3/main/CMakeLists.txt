idf_component_register(SRCS "main.c")

add_prebuilt_library(prebuilt "libprebuilt.a")

target_link_libraries(${COMPONENT_LIB} PRIVATE prebuilt)
