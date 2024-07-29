add_library(CartesioManipulabilityAddon SHARED
    src/Manipulability.cpp
    src/OpenSotManipulability.cpp
)

target_link_libraries(CartesioManipulabilityAddon PRIVATE
    ${OpenSoT_LIBRARIES}
    CartesianInterface)

install(TARGETS CartesioManipulabilityAddon
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
