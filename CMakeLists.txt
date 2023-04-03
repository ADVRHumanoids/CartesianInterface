cmake_minimum_required(VERSION 3.0)
project(cartesian_interface VERSION 2.1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(XBOT2_ENABLE_XENO OFF "Compile against xenomai")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(XBotInterface REQUIRED)
find_package(OpenSoT REQUIRED)
find_package(ReflexxesTypeII REQUIRED)
find_package(matlogger2 REQUIRED)
find_package(Eigen3 REQUIRED)
set(CATKIN_BUILD_BINARY_PACKAGE TRUE)
find_package(catkin REQUIRED COMPONENTS roscpp
    std_msgs
    xbot_msgs
    geometry_msgs
    sensor_msgs
    actionlib
    robot_state_publisher
    message_generation
    interactive_markers
    tf tf_conversions)

find_package(GLPK QUIET)
if(${GLPK_FOUND})
    message("glpk found!")
    add_definitions( -D_GLPK_FOUND=${GLPK_FOUND} )
endif()


option(CARTESIO_USE_ALGLIB "Support spline trajectories with alglib" OFF)
if(${CARTESIO_USE_ALGLIB})
    find_package(alglib QUIET)
endif()

include_directories(include
    include/${PROJECT_NAME}/external
    include/${PROJECT_NAME}/sdk
    src
    ${OpenSoT_INCLUDE_DIRS}
    ${XBotInterface_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${ReflexxesTypeII_INCLUDE_DIRS}/..
    ${alglib_INCLUDE_DIRS}
    )


# C++ 11
add_compile_options(-std=c++14)
add_compile_options(-Wl,--no-undefined)

# Generate messages in the 'msg' folder
add_message_files(
    FILES
    Impedance3.msg
    Impedance6.msg
    CartesianImpedanceBase.msg
    CartesianImpedance.msg
    CartesianImpedanceTimed.msg
    JoystickStatus.msg
    TaskInfo.msg
    CartesianTaskInfo.msg
    InteractionTaskInfo.msg
    )

# Generate services in the 'srv' folder
add_service_files(
    FILES
    GetTaskInfo.srv
    SetTaskActive.srv
    LoadController.srv
    GetTaskList.srv
    ResetJoints.srv
    ResetWorld.srv
    GetImpedance.srv
    SetImpedance.srv
    ToggleAxis.srv
    SetContactFrame.srv
    SetJoystickActiveTask.srv
    SetJoystickTaskMaxSpeed.srv
    SetJoystickTaskBaseFrame.srv
    GetCartesianTaskInfo.srv
    GetInteractionTaskInfo.srv
    SetWeight.srv
    SetLambda.srv
    SetLambda2.srv
    SetBaseLink.srv
    SetControlMode.srv
    SetSafetyLimits.srv
    SetTransform.srv
    )


# Generate actions in the 'action' folder
add_action_files(
    FILES
    ReachPose.action
    ReachCartesianImpedance.action
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
    SKIP_CMAKE_CONFIG_GENERATION
    SKIP_PKG_CONFIG_GENERATION
    )

# Interface implementation
set(CartesianInterface_SRC
    src/Context.cpp
    src/CartesianInterfaceImpl.cpp
    src/utils/LockfreeBufferImpl.cpp
    src/utils/ManipulabilityAnalyzer.cpp
    src/problem/ProblemDescription.cpp
    src/problem/impl/Cartesian.cpp
    src/problem/impl/Com.cpp
    src/problem/impl/Limits.cpp
    #        src/problem/Gaze.cpp
    src/problem/impl/Postural.cpp
    src/problem/impl/Subtask.cpp
    src/problem/impl/Interaction.cpp
    src/problem/impl/TaskFactory.cpp
    src/problem/impl/Task.cpp
    src/problem/impl/Constraint.cpp
    src/rt/TaskRt.cpp
    src/rt/CartesianRt.cpp
    src/rt/CInteractionRt.cpp
    src/utils/ForceEstimation.cpp
    src/utils/RobotStatePublisher.cpp
    #        src/problem/MinJointVel.cpp
    )

# OpenSot implementation
set(CartesianInterfaceSOT_SRC
    src/opensot/task_adapters/OpenSotTask.cpp
    src/opensot/task_adapters/OpenSotCartesian.cpp
    src/opensot/task_adapters/OpenSotInteraction.cpp
    src/opensot/task_adapters/OpenSotCom.cpp
    src/opensot/task_adapters/OpenSotPostural.cpp
    src/opensot/task_adapters/OpenSotSubtask.cpp
    src/opensot/task_adapters/OpenSotJointLimits.cpp
    src/opensot/task_adapters/OpenSotVelocityLimits.cpp
    src/opensot/task_adapters/OpenSotConstraintFromTask.cpp
    src/opensot/OpenSotImpl.cpp
    )

# Ros implementation
set(CartesianInterfaceROS_SRC
    src/ros/RosContext.cpp
    src/ros/server_api/TaskRos.cpp
    src/ros/server_api/CartesianRos.cpp
    src/ros/server_api/InteractionRos.cpp
    src/ros/server_api/PosturalRos.cpp
    src/ros/client_api/TaskRos.cpp
    src/ros/client_api/CartesianRos.cpp
    src/ros/client_api/InteractionRos.cpp
    src/ros/client_api/PosturalRos.cpp
    src/ros/RosImpl.cpp
    src/ros/RosServerClass.cpp
    src/ros/CartesianMarker.cpp
    src/ros/JoyStick.cpp
    #        src/ros/CartesianAnalyzer.cpp
    src/ros/RosExecutor.cpp
    )

# Trajectory
set(Trajectory_SRC src/trajectory/Trajectory.cpp)

if(${alglib_FOUND})
    set(Trajectory_SRC ${Trajectory_SRC}
        src/trajectory/Spline.cpp)
endif()

# libCartesianInterface
add_library(CartesianInterface SHARED
    ${CartesianInterface_SRC}
    ${CartesianInterfaceROS_SRC}
    ${Trajectory_SRC}
    ${CartesianInterfaceSOT_SRC}

    )

target_link_libraries(CartesianInterface
    PUBLIC
    XBotInterface::XBotInterface
    OpenSoT::OpenSoT
    matlogger2::matlogger2
    ${catkin_LIBRARIES}
    yaml-cpp
    PRIVATE
    ${ReflexxesTypeII_LIBRARIES}
    ${alglib_LIBRARIES}

    )


target_compile_definitions(CartesianInterface
    PUBLIC -DFMT_HEADER_ONLY=1)

target_include_directories(CartesianInterface
    PUBLIC
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/external>
    )

add_dependencies(CartesianInterface ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS CartesianInterface
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )



# libCartesianOpenSot
add_library(CartesianInterfaceSolverOpenSot SHARED
    src/opensot/OpenSotImpl.cpp)

target_link_libraries(CartesianInterfaceSolverOpenSot PUBLIC
    ${OpenSoT_LIBRARIES}
    CartesianInterface
    PRIVATE
    matlogger2::matlogger2
    )
install(TARGETS CartesianInterfaceSolverOpenSot
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
target_link_libraries(torque_feedforward_node ${OpenSoT_LIBRARIES} CartesianInterface)
install(TARGETS torque_feedforward_node torque_feedforward_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Robot state publisher executable
add_executable(robot_state_publisher src/utils/RobotStatePublisherNode.cpp)
target_link_libraries(robot_state_publisher CartesianInterface)
install(TARGETS robot_state_publisher robot_state_publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

option(CARTESIO_COMPILE_EXAMPLES "Compile examples" OFF)
if(${CARTESIO_COMPILE_EXAMPLES})
    add_subdirectory(examples)
endif()

add_subdirectory(bindings/python/)

include(GenerateDeb)


option(CARTESIO_COMPILE_XBOT_PLUGIN "Build plugins for XBotCore" OFF)

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

include(ExportCartesianInterface)
export_cartesian_interface()

#######################
# Add Testing target  #
#######################
option(CARTESIO_BUILD_TESTS "Compile CartesianInterface tests" FALSE)

if(CARTESIO_BUILD_TESTS)
    enable_testing()
    add_custom_target(test_verbose ${CMAKE_CTEST_COMMAND} -V)
    add_subdirectory(tests)
endif()
