add_subdirectory(plugin)
add_subdirectory(cpp)

install(DIRECTORY configs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples/configs
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY mesh/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples/mesh
)

install(DIRECTORY srdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples/srdf
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples/urdf
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples/rviz
)

install(DIRECTORY python/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples/python
)

