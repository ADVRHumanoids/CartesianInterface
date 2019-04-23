project(cartesian_interface)
cmake_minimum_required(VERSION 3.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroInstallLib)

find_package(XBotInterface REQUIRED)
find_package(XCM QUIET)
find_package(OpenSoT REQUIRED)
find_package(ReflexxesTypeII REQUIRED)
find_package(alglib QUIET)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs
                                        geometry_msgs
                                        sensor_msgs
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
                    ${ReflexxesTypeII_INCLUDE_DIRS}
                    ${alglib_INCLUDE_DIRS}
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
  Impedance3.msg 
  Impedance6.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  SetTaskInfo.srv
  GetTaskInfo.srv
  LoadController.srv
  GetTaskList.srv
  ResetWorld.srv
  GetImpedance.srv
  ToggleAxis.srv
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
  sensor_msgs
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
   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
   LIBRARIES CartesianInterface
#  CATKIN_DEPENDS other_catkin_pkg
   DEPENDS XBotInterface yaml-cpp
   CFG_EXTRAS ${PROJECT_NAME}-extras.cmake
)

# Interface implementation
set(CartesianInterface_SRC
        src/CartesianInterfaceImpl.cpp
        src/utils/LockfreeBufferImpl.cpp
        src/utils/ManipulabilityAnalyzer.cpp
        src/problem/ProblemDescription.cpp
        src/problem/Cartesian.cpp
        src/problem/Com.cpp
        src/problem/Gaze.cpp
        src/problem/AngularMomentum.cpp
        src/problem/Postural.cpp
        src/problem/Interaction.cpp
        src/problem/TaskFactory.cpp
        src/problem/TaskImpl.cpp
        src/problem/ConstraintImpl.cpp
        src/utils/ForceEstimation.cpp
        
    )

# OpenSot implementation
set(CartesianInterfaceSOT_SRC
        src/open_sot/OpenSotImpl.cpp
    )

# Ros implementation
set(CartesianInterfaceROS_SRC
        src/ros/RosImpl.cpp
        src/ros/RosServerClass.cpp
        src/ros/CartesianMarker.cpp
        src/ros/JoyStick.cpp
        src/ros/Aeronav.cpp
        src/ros/CartesianAnalyzer.cpp
        src/ros/RosExecutor.cpp
    )

# Trajectory
set(Trajectory_SRC src/trajectory/Trajectory.cpp)
    
if(${alglib_FOUND})
    set(Trajectory_SRC ${Trajectory_SRC}
                       src/trajectory/Spline.cpp)
endif()

# should we use xbot internal robot state publisher?
option(USE_XBOT_ROBOT_STATE_PUB FALSE)
set(XBOT_RSPUB_LIBRARIES "")
if(${USE_XBOT_ROBOT_STATE_PUB})
    add_definitions(-DXBOT_RSPUB)
    find_package(robot_state_publisher_advr_solver REQUIRED)
    include_directories(${robot_state_publisher_advr_solver_INCLUDE_DIRS})
    set(XBOT_RSPUB_LIBRARIES ${robot_state_publisher_advr_solver_LIBRARIES})
endif()

# libCartesianInterface
add_library(CartesianInterface SHARED ${CartesianInterface_SRC}
                                      ${CartesianInterfaceROS_SRC}
                                      ${Trajectory_SRC}
                                      )
target_link_libraries(CartesianInterface PUBLIC ${OpenSoT_LIBRARIES}
                                                ${XBotInterface_LIBRARIES}
                                                ${catkin_LIBRARIES}
                                                ${XBOT_RSPUB_LIBRARIES}
                                                yaml-cpp
                                                ${ReflexxesTypeII_LIBRARIES}
                                                ${alglib_LIBRARIES}
                      )
add_dependencies(CartesianInterface ${PROJECT_NAME}_generate_messages)

install(TARGETS CartesianInterface CartesianInterface
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



# libCartesianOpenSot
add_library(CartesianOpenSot SHARED ${CartesianInterfaceSOT_SRC})
target_link_libraries(CartesianOpenSot PUBLIC ${OpenSoT_LIBRARIES}
                                              CartesianInterface
                      )
install(TARGETS CartesianOpenSot CartesianOpenSot
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Addons for libCartesianOpenSot

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_library(CartesioConvexHullAddon SHARED src/open_sot/ConvexHullAddon.cpp)
target_link_libraries(CartesioConvexHullAddon PRIVATE ${OpenSoT_LIBRARIES}
                                                      CartesianInterface
                                                      ${PCL_LIBRARIES})
install(TARGETS CartesioConvexHullAddon CartesioConvexHullAddon
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



# Cartesian Server executable
add_executable(ros_server_node src/ros/RosServerNode.cpp)
target_link_libraries(ros_server_node CartesianInterface dl)
install(TARGETS ros_server_node ros_server_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Marker spawner executable
add_executable(marker_spawner src/ros/MarkerSpawner.cpp)
target_link_libraries(marker_spawner CartesianInterface)
install(TARGETS marker_spawner marker_spawner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# JoyStick spawner executable
add_executable(joystick_spawner src/ros/JoyStickSpawner.cpp)
target_link_libraries(joystick_spawner CartesianInterface)
install(TARGETS joystick_spawner joystick_spawner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Aeronav spawner executable
add_executable(aeronav_spawner src/ros/AeronavSpawner.cpp)
target_link_libraries(aeronav_spawner CartesianInterface)
install(TARGETS aeronav_spawner aeronav_spawner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# CartesianAnalyzer executable
add_executable(cartesian_analyzer src/ros/CartesianAnalyzer.cpp)
target_link_libraries(cartesian_analyzer CartesianInterface)
install(TARGETS cartesian_analyzer cartesian_analyzer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ForceEstimationNode executable
add_executable(force_estimation_node src/ros/ForceEstimationNode.cpp)
target_link_libraries(force_estimation_node CartesianInterface)
install(TARGETS force_estimation_node force_estimation_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# TorqueFeedforward executable
add_executable(torque_feedforward_node src/ros/TorqueFeedforward.cpp)
target_link_libraries(torque_feedforward_node CartesianInterface)
install(TARGETS torque_feedforward_node torque_feedforward_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# WorldConnecter executable
add_executable(world_connecter src/ros/WorldConnecter.cpp)
target_link_libraries(world_connecter ${catkin_LIBRARIES})
install(TARGETS world_connecter world_connecter
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_subdirectory(examples)
add_subdirectory(bindings/python/)


option(COMPILE_XBOT_PLUGIN OFF)

if(${XCM_FOUND} AND ${COMPILE_XBOT_PLUGIN})
    add_subdirectory(xbotcore)
endif()


# Mark cpp header files for installation
install(DIRECTORY include/cartesian_interface/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h*"
    PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)



install(PROGRAMS src/postural_gui DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#######################
# Add Testing target  #
#######################
option(BUILD_TESTS "Compile CartesianInterface tests" FALSE)

if(BUILD_TESTS)
  enable_testing()
  add_custom_target(test_verbose ${CMAKE_CTEST_COMMAND} -V)
  add_subdirectory(tests)
endif()
