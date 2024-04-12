find_package(xbot2 REQUIRED)

add_xbot2_plugin(cartesio_plugin cartesioplugin.cpp)

target_link_libraries(cartesio_plugin PUBLIC CartesianInterface)

install(TARGETS cartesio_plugin
    DESTINATION lib)
