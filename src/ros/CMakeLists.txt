
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

# ros1 source files
set(CartesianInterfaceRos_SRC
    RosContext.cpp
    server_api/TaskRos.cpp
    server_api/CartesianRos.cpp
    server_api/InteractionRos.cpp
    server_api/PosturalRos.cpp
    client_api/TaskRos.cpp
    client_api/CartesianRos.cpp
    client_api/InteractionRos.cpp
    client_api/PosturalRos.cpp
    RosImpl.cpp
    RosServerClass.cpp
    CartesianMarker.cpp
    JoyStick.cpp
    RosExecutor.cpp
    utils/RobotStatePublisher.cpp
    )

add_library(CartesianInterfaceRos STATIC
    ${CartesianInterfaceRos_SRC}
    )

add_dependencies(CartesianInterfaceRos ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Cartesian Server executable
add_executable(ros_server_node RosServerNode.cpp)
target_link_libraries(ros_server_node CartesianInterface dl)
install(TARGETS ros_server_node ros_server_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# Marker spawner executable
add_executable(marker_spawner MarkerSpawner.cpp)
target_link_libraries(marker_spawner CartesianInterface)
install(TARGETS marker_spawner marker_spawner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# JoyStick spawner executable
add_executable(joystick_spawner JoyStickSpawner.cpp)
target_link_libraries(joystick_spawner CartesianInterface)
install(TARGETS joystick_spawner joystick_spawner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# CartesianAnalyzer executable
add_executable(cartesian_analyzer CartesianAnalyzer.cpp)
target_link_libraries(cartesian_analyzer CartesianInterface)
install(TARGETS cartesian_analyzer cartesian_analyzer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Robot state publisher executable
add_executable(robot_state_publisher utils/RobotStatePublisherNode.cpp)
target_link_libraries(robot_state_publisher CartesianInterface)
install(TARGETS robot_state_publisher robot_state_publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
    )

install(PROGRAMS ${CMAKE_SOURCE_DIR}/src/postural_gui DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
