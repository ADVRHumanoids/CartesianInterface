
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)

# Generate messages in the 'msg' folder
set(msg_files
    msg/Impedance3.msg
    msg/Impedance6.msg
    msg/CartesianImpedanceBase.msg
    msg/CartesianImpedance.msg
    msg/CartesianImpedanceTimed.msg
    msg/JoystickStatus.msg
    msg/TaskInfo.msg
    msg/CartesianTaskInfo.msg
    msg/InteractionTaskInfo.msg
)

# Generate services in the 'srv' folder
set(srv_files
    srv/GetTaskInfo.srv
    srv/SetTaskActive.srv
    srv/LoadController.srv
    srv/GetTaskList.srv
    srv/ResetJoints.srv
    srv/ResetWorld.srv
    srv/GetImpedance.srv
    srv/SetImpedance.srv
    srv/ToggleAxis.srv
    srv/SetContactFrame.srv
    srv/SetJoystickActiveTask.srv
    srv/SetJoystickTaskMaxSpeed.srv
    srv/SetJoystickTaskBaseFrame.srv
    srv/GetCartesianTaskInfo.srv
    srv/GetInteractionTaskInfo.srv
    srv/SetWeight.srv
    srv/SetLambda.srv
    srv/SetLambda2.srv
    srv/SetBaseLink.srv
    srv/SetControlMode.srv
    srv/SetSafetyLimits.srv
    srv/SetTransform.srv
)


# Generate actions in the 'action' folder
set(action_files
    action/ReachPose.action
    action/ReachCartesianImpedance.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
    DEPENDENCIES
    builtin_interfaces
    sensor_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()


# ros source files
set(CartesianInterfaceRos_SRC
    RosContext.cpp
    server_api/TaskRos.cpp
    server_api/CartesianRos.cpp
    #    server_api/InteractionRos.cpp
    server_api/PosturalRos.cpp
    client_api/TaskRos.cpp
    client_api/CartesianRos.cpp
    #    client_api/InteractionRos.cpp
    #    client_api/PosturalRos.cpp
    #    RosImpl.cpp
    RosServerClass.cpp
    CartesianMarker.cpp
    #    JoyStick.cpp
    RosExecutor.cpp
    utils/RobotStatePublisher.cpp
)

add_library(CartesianInterfaceRos STATIC
    ${CartesianInterfaceRos_SRC}
    utils/RosUtils.h
)

target_link_libraries(CartesianInterfaceRos
    PUBLIC
    CartesianInterfaceCore
    rclcpp::rclcpp
    tf2_eigen::tf2_eigen
    tf2_kdl::tf2_kdl
    tf2_geometry_msgs::tf2_geometry_msgs
    std_srvs::std_srvs__rosidl_typesupport_cpp
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    interactive_markers::interactive_markers
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
)

#add_dependencies(CartesianInterfaceRos ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Cartesian Server executable
add_executable(ros_server_node RosServerNode.cpp)

target_link_libraries(ros_server_node CartesianInterfaceRos rclcpp::rclcpp dl)

install(TARGETS ros_server_node
    DESTINATION lib/${PROJECT_NAME}
)

# Marker spawner executable
add_executable(marker_spawner
    MarkerSpawner.cpp
)

target_link_libraries(marker_spawner
    CartesianInterfaceRos
)

install(TARGETS marker_spawner
   DESTINATION lib/${PROJECT_NAME}
   )

# Robot description publisher
add_executable(robot_description_publisher
    RobotDescriptionPublisher.cpp
)

target_link_libraries(robot_description_publisher
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
)

install(TARGETS robot_description_publisher
   DESTINATION lib/${PROJECT_NAME}
   )

## JoyStick spawner executable
#add_executable(joystick_spawner JoyStickSpawner.cpp)
#target_link_libraries(joystick_spawner CartesianInterface)
#install(TARGETS joystick_spawner joystick_spawner
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## CartesianAnalyzer executable
#add_executable(cartesian_analyzer CartesianAnalyzer.cpp)
#target_link_libraries(cartesian_analyzer CartesianInterface)
#install(TARGETS cartesian_analyzer cartesian_analyzer
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Robot state publisher executable
#add_executable(robot_state_publisher utils/RobotStatePublisherNode.cpp)
#target_link_libraries(robot_state_publisher CartesianInterface)
#install(TARGETS robot_state_publisher robot_state_publisher
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

### Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY ${CMAKE_SOURCE_DIR}/launch/
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#    FILES_MATCHING PATTERN "*.launch"
#    )

#install(PROGRAMS ${CMAKE_SOURCE_DIR}/src/postural_gui DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
