
find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
find_package(pybind11 QUIET)

message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")

    # get python install path
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "import sys; version = sys.version_info[:3]; print('{0}.{1}'.format(*version))"
        OUTPUT_VARIABLE PYTHON_VERSION_MIN
        OUTPUT_STRIP_TRAILING_WHITESPACE)

    set(PYTHON_SITE "lib/python${PYTHON_VERSION_MIN}/site-packages")

    message(STATUS "Python install dir: ${PYTHON_SITE}")

    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pyest pyForceEstimation.cpp)
    target_link_libraries(pyest PUBLIC estimation_utils::payload_estimation)

    install(TARGETS pyest
            DESTINATION ${PYTHON_SITE})

#    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#    pybind11_add_module(pyci pyRosImpl.cpp)
#    target_link_libraries(pyci PRIVATE CartesianInterface)

#    install(TARGETS pyci
#            DESTINATION ${PYTHON_SITE})

#    pybind11_add_module(roscpp_utils pyRosInit.cpp)
#    target_link_libraries(roscpp_utils PRIVATE ${catkin_LIBRARIES})

#    install(TARGETS roscpp_utils
#            DESTINATION ${PYTHON_SITE})



    pybind11_add_module(impedance pyImpedance.cpp)
    target_link_libraries(impedance PRIVATE CartesianInterfaceCore)

    install(TARGETS impedance
            DESTINATION ${PYTHON_SITE})

    install(FILES pyci_all.py
            DESTINATION ${PYTHON_SITE})

    install(PROGRAMS interactive_client.py
            DESTINATION ${PYTHON_SITE})

else()
    message(STATUS "pybind not found")
endif()
