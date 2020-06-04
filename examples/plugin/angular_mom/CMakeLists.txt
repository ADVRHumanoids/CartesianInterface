project(cartesian_interface_angmom_plugin)

#AngularMomentumAddon (Velocity)
add_library(CartesioAngularMomentumAddon SHARED
    src/AngularMomentum.cpp
    src/OpenSotAngularMomentum.cpp
    src/AngularMomentumRos.cpp)

target_link_libraries(CartesioAngularMomentumAddon PRIVATE
    ${OpenSoT_LIBRARIES}
    CartesianInterface)

install(TARGETS CartesioAngularMomentumAddon CartesioAngularMomentumAddon
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
