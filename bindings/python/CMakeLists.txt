cmake_minimum_required(VERSION 3.5)

find_package(pybind11 QUIET)
message(STATUS "**********************************Check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "+++++++++++++++++++++++++++++++Compiling python bindings")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyest pyForceEstimation.cpp)
    target_link_libraries(pyest PUBLIC CartesianInterface)

    install(TARGETS pyest
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyci pyRosImpl.cpp)
    target_link_libraries(pyci PRIVATE CartesianInterface)

    install(TARGETS pyci
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})


    pybind11_add_module(roscpp_utils pyRosInit.cpp)
    target_link_libraries(roscpp_utils PRIVATE ${catkin_LIBRARIES})

    install(TARGETS roscpp_utils
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
            
            
    pybind11_add_module(affine3 pyAffine3d.cpp)

    install(TARGETS affine3
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
            
    install(FILES pyci_all.py
            DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
            
else()
    message(STATUS "-----------------------------------pybind not found")
endif()
