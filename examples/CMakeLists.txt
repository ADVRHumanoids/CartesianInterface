cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

set(PROJECTNAME examples)
project(${PROJECTNAME})

find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs
                                        geometry_msgs
                                        actionlib
                                        robot_state_publisher
                                        tf tf_conversions)

add_executable(Cartesian_marker_node CartesianMarkerNode.cpp)
target_link_libraries(Cartesian_marker_node CartesianInterface)



install(TARGETS Cartesian_marker_node Cartesian_marker_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      )
