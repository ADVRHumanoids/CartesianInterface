cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

project(cartesian_interface_tests C CXX)


# compite GTest
find_package(GTest)
find_package(Threads)
if(NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

add_definitions(-DCARTESIO_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/configs/")
add_definitions(-DCARTESIO__TEST_BINARY_PATH="${CMAKE_BINARY_DIR}/devel/lib/cartesian_interface")

set(TestLibs CartesianInterface ${GTEST_BOTH_LIBRARIES})
include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

add_executable(TestTask TestTask.cpp)
target_link_libraries(TestTask ${TestLibs})
add_dependencies(TestTask GTest-ext CartesianInterface)
add_test(NAME TestTask COMMAND TestTask)

add_executable(TestCartesian TestCartesian.cpp)
target_link_libraries(TestCartesian ${TestLibs})
add_dependencies(TestCartesian GTest-ext CartesianInterface)
add_test(NAME TestCartesian COMMAND TestCartesian)

add_executable(TestApi TestApi.cpp)
target_link_libraries(TestApi ${TestLibs})
add_dependencies(TestApi GTest-ext CartesianInterface)
add_test(NAME TestApi COMMAND TestApi)

add_executable(TestOpensot TestOpensot.cpp)
target_link_libraries(TestOpensot ${TestLibs} dl)
add_dependencies(TestOpensot GTest-ext CartesianInterface)
add_test(NAME TestOpensot COMMAND TestOpensot)

add_executable(TestRosClient TestRosClient.cpp)
target_link_libraries(TestRosClient ${TestLibs} fmt::fmt-header-only)
add_dependencies(TestRosClient GTest-ext CartesianInterface)
add_test(NAME TestRosClient COMMAND TestRosClient)

add_executable(TestRt TestRt.cpp)
target_link_libraries(TestRt dl ${TestLibs} fmt::fmt-header-only)
add_dependencies(TestRt GTest-ext CartesianInterface)
add_test(NAME TestRt COMMAND TestRt)

add_executable(TestRos TestRos.cpp)
target_link_libraries(TestRos ${TestLibs} dl fmt::fmt-header-only)
add_dependencies(TestRos GTest-ext CartesianInterface )
add_test(NAME TestRos COMMAND TestRos)

