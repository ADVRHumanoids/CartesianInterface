project(cartesian_interface)
cmake_minimum_required(VERSION 3.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroInstallLib)

find_package(XBotInterface REQUIRED)
find_package(XCM QUIET)
find_package(OpenSoT REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs
                                        geometry_msgs
                                        actionlib
                                        robot_state_publisher
                                        message_generation
                                        interactive_markers
                                        tf tf_conversions)

include_directories(include
                    ${OpenSoT_INCLUDE_DIRS}
                    ${XBotInterface_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    )


# C++ 11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ReferenceStamped.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  TaskList.srv
  SetTaskInfo.srv
  GetTaskInfo.srv
  LoadController.srv
)

# Generate actions in the 'action' folder
add_action_files(
  FILES
  ReachPose.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES CartesianInterface
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Interface implementation
set(CartesianInterface_SRC
        src/CartesianInterfaceImpl.cpp
        src/ProblemDescription.cpp
    )

# OpenSot implementation
set(CartesianInterfaceSOT_SRC
        src/OpenSotImpl.cpp
    )

# Ros implementation
set(CartesianInterfaceROS_SRC
        src/RosImpl.cpp
        src/RosServerClass.cpp
        src/CartesianMarker.cpp
    )

# Trajectory
set(Trajectory_SRC
        src/Trajectory.cpp
    )

# libCartesianInterface
add_library(CartesianInterface SHARED ${CartesianInterface_SRC}
                                      ${CartesianInterfaceROS_SRC}
                                      ${Trajectory_SRC}
                                      )
target_link_libraries(CartesianInterface PUBLIC ${OpenSoT_LIBRARIES}
                                                ${XBotInterface_LIBRARIES}
                                                ${catkin_LIBRARIES}
                                                yaml-cpp
                      )
add_dependencies(CartesianInterface ${PROJECT_NAME}_generate_messages)


# libCartesianOpenSot
add_library(CartesianOpenSot SHARED ${CartesianInterfaceSOT_SRC})
target_link_libraries(CartesianOpenSot PUBLIC ${OpenSoT_LIBRARIES}
                                              CartesianInterface
                      )




# Ros server
add_executable(ros_server_node src/RosServerNode.cpp)
target_link_libraries(ros_server_node CartesianInterface dl)

add_subdirectory(examples)
add_executable(solver_test src/BackendExample.cpp)
target_link_libraries(solver_test ${OpenSoT_LIBRARIES})

option(COMPILE_XBOT_PLUGIN OFF)

if(${XCM_FOUND} AND ${COMPILE_XBOT_PLUGIN})
    add_subdirectory(xbotcore)
endif()

# Mark executables and/or libraries for installation
install(TARGETS ros_server_node ros_server_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS CartesianInterface CartesianInterface
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS CartesianOpenSot CartesianOpenSot
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/cartesian_interface/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#######################
# Add Testing target  #
#######################
option(BUILD_TESTS "Compile CartesianInterface tests" FALSE)

if(BUILD_TESTS)
  enable_testing()
  add_custom_target(test_verbose ${CMAKE_CTEST_COMMAND} -V)
  add_subdirectory(tests)
endif()
