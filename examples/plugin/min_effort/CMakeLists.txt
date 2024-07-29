add_library(CartesioMinimumEffortAddon SHARED
    src/MinEffort.cpp
    src/OpenSotMinEffort.cpp
)

target_link_libraries(CartesioMinimumEffortAddon PRIVATE
    ${OpenSoT_LIBRARIES}
    CartesianInterface)

install(TARGETS CartesioMinimumEffortAddon
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
