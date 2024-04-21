cmake_minimum_required(VERSION 3.5)

#if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
#    message("Please consider to switch to latest version to enable
#    more reliable Python3 binaries detection")
#else()
#    find_package(Python${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
#endif()

set(PYBIND11_PYTHON_VERSION 3)
find_package(pybind11 QUIET)

message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings -> ${PYTHON_EXECUTABLE}")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyest pyForceEstimation.cpp)
    target_link_libraries(pyest PUBLIC CartesianInterface)

    install(TARGETS pyest
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyci pyRosImpl.cpp)
    target_link_libraries(pyci PRIVATE CartesianInterface CartesioAngularMomentumAddon)

    install(TARGETS pyci
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

    pybind11_add_module(roscpp_utils pyRosInit.cpp)
    target_link_libraries(roscpp_utils PRIVATE ${catkin_LIBRARIES})

    install(TARGETS roscpp_utils
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
            
            
    pybind11_add_module(affine3 pyAffine3d.cpp)

    install(TARGETS affine3
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

    pybind11_add_module(impedance pyImpedance.cpp)
    target_link_libraries(impedance PRIVATE CartesianInterface)

    install(TARGETS impedance
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
            
    install(FILES pyci_all.py
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

    install(PROGRAMS interactive_client.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
            
else()
    message(STATUS "pybind not found")
endif()
