#AngularMomentumAddon (Velocity)
add_library(CartesioAngularMomentumAddon SHARED
    src/AngularMomentum.cpp
    src/OpenSotAngularMomentum.cpp
    src/AngularMomentumRos.cpp)

target_link_libraries(CartesioAngularMomentumAddon PRIVATE
    ${OpenSoT_LIBRARIES}
    CartesianInterface)

install(TARGETS CartesioAngularMomentumAddon
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark cpp header files for installation
install(FILES "src/AngularMomentum.h" "src/AngularMomentumRos.h"
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
