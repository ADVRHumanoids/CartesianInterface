cmake_minimum_required(VERSION 3.5)

find_package(pybind11 QUIET)

if(${pybind11_FOUND})

    message(STATUS "Compiling python bindings")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyci_ros pyRosImpl.cpp)
    target_link_libraries(pyci_ros PRIVATE CartesianInterface)

    install(TARGETS pyci_ros
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

            
    pybind11_add_module(roscpp_utils pyRosInit.cpp)
    target_link_libraries(roscpp_utils PRIVATE ${catkin_LIBRARIES})

    install(TARGETS roscpp_utils
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
            
endif()