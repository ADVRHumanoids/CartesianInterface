#AngularMomentumAddon (Velocity)
add_library(CartesioAngularMomentumAddon SHARED
    src/AngularMomentum.cpp
    src/OpenSotAngularMomentum.cpp
    src/AngularMomentumRos.cpp)

target_link_libraries(CartesioAngularMomentumAddon PRIVATE
    OpenSoT::OpenSoT
    CartesianInterface
    CartesianInterfaceRos)

install(TARGETS CartesioAngularMomentumAddon
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    )
