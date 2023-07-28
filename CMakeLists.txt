cmake_minimum_required(VERSION 3.10)
project(cartesian_interface VERSION 2.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# required packages
find_package(XBotInterface REQUIRED)
find_package(OpenSoT REQUIRED)
find_package(ReflexxesTypeII REQUIRED)
find_package(estimation_utils REQUIRED)

# optional packages
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
    ${ReflexxesTypeII_INCLUDE_DIRS}/..
    ${alglib_INCLUDE_DIRS}
    )


# core sources
set(CartesianInterface_SRC
    src/Context.cpp
    src/CartesianInterfaceImpl.cpp
    src/problem/ProblemDescription.cpp
    src/problem/impl/Cartesian.cpp
    src/problem/impl/Com.cpp
    src/problem/impl/Limits.cpp
    src/problem/impl/Postural.cpp
    src/problem/impl/Subtask.cpp
    src/problem/impl/Interaction.cpp
    src/problem/impl/TaskFactory.cpp
    src/problem/impl/Task.cpp
    src/problem/impl/Constraint.cpp
#    src/utils/LockfreeBufferImpl.cpp
#    src/rt/TaskRt.cpp
#    src/rt/CartesianRt.cpp
#    src/rt/CInteractionRt.cpp
    )

# opensot sources
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

# trajectory sources
set(Trajectory_SRC src/trajectory/Trajectory.cpp)

if(${alglib_FOUND})
    set(Trajectory_SRC ${Trajectory_SRC}
        src/trajectory/Spline.cpp)
endif()

# compile core library
add_library(CartesianInterfaceCore STATIC
    ${CartesianInterface_SRC}
    ${Trajectory_SRC}
    )

target_link_libraries(CartesianInterfaceCore
    PUBLIC
    OpenSoT::OpenSoT
    yaml-cpp
    PRIVATE
    ${ReflexxesTypeII_LIBRARIES}
    ${alglib_LIBRARIES}
    )

target_compile_definitions(CartesianInterfaceCore
    PUBLIC -DFMT_HEADER_ONLY=1)

target_include_directories(CartesianInterfaceCore
    PUBLIC
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/external>
    )

# link shared library with core and ros
add_library(CartesianInterface SHARED
    src/empty.cpp
    )

# compile ros library
#add_subdirectory(src/ros)

#target_link_libraries(CartesianInterface
#    PRIVATE
#    "-Wl,--whole-archive"
#    CartesianInterfaceRos
#    "-Wl,--no-whole-archive"
#    )

target_link_libraries(CartesianInterface
    PUBLIC
    "-Wl,--whole-archive"
    CartesianInterfaceCore
    "-Wl,--no-whole-archive"
    )

install(TARGETS CartesianInterface
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    )

install(TARGETS CartesianInterfaceCore
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    )

# compile opensot implementation
add_library(CartesianInterfaceSolverOpenSot SHARED
    ${CartesianInterfaceSOT_SRC}
    src/opensot/OpenSotImpl.cpp
    )

target_link_libraries(CartesianInterfaceSolverOpenSot
    PRIVATE
    OpenSoT::OpenSoT
    CartesianInterfaceCore
    estimation_utils::payload_estimation
    matlogger2::matlogger2
    )
install(TARGETS CartesianInterfaceSolverOpenSot
    LIBRARY DESTINATION lib
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
    DESTINATION include/cartesian_interface
    FILES_MATCHING PATTERN "*.h*"
    PATTERN ".svn" EXCLUDE
    )



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
