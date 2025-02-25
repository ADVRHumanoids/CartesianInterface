
find_package(Python ${PYBIND11_PYTHON_VERSION} COMPONENTS Development Interpreter REQUIRED)
find_package(pybind11 QUIET)

message(STATUS "check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "compiling python bindings")

    # get python install path
    message(STATUS "Found python (${Python3_EXECUTABLE}) will compile python bindings")

    # get python install path
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import site, os; print(os.path.relpath(site.USER_SITE, site.USER_BASE))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)

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
