include(ExternalProject)

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
    set(GTEST_DEPENDS GTest-ext)
endif()

add_definitions(-DCARTESIO_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/configs/")
add_definitions(-DCARTESIO__TEST_BINARY_PATH="${CMAKE_BINARY_DIR}/devel/lib/cartesian_interface")

set(TestLibs CartesianInterface ${GTEST_BOTH_LIBRARIES})
include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

add_executable(TestTask TestTask.cpp)
target_link_libraries(TestTask ${TestLibs})
add_dependencies(TestTask ${GTEST_DEPENDS} CartesianInterface)
add_test(NAME TestTask COMMAND TestTask)

add_executable(TestCartesian TestCartesian.cpp)
target_link_libraries(TestCartesian ${TestLibs})
add_dependencies(TestCartesian ${GTEST_DEPENDS} CartesianInterface)
add_test(NAME TestCartesian COMMAND TestCartesian)

add_executable(TestApi TestApi.cpp)
target_link_libraries(TestApi ${TestLibs})
add_dependencies(TestApi ${GTEST_DEPENDS} CartesianInterface)
add_test(NAME TestApi COMMAND TestApi)

add_executable(TestOpensot TestOpensot.cpp)
target_link_libraries(TestOpensot ${TestLibs} dl)
add_dependencies(TestOpensot ${GTEST_DEPENDS} CartesianInterface)
add_test(NAME TestOpensot COMMAND TestOpensot)

add_executable(TestRosClient TestRosClient.cpp)
target_link_libraries(TestRosClient ${TestLibs})
add_dependencies(TestRosClient ${GTEST_DEPENDS} CartesianInterface)
add_test(NAME TestRosClient COMMAND TestRosClient)

add_executable(TestRt TestRt.cpp)
target_link_libraries(TestRt dl ${TestLibs})
add_dependencies(TestRt ${GTEST_DEPENDS} CartesianInterface)
add_test(NAME TestRt COMMAND TestRt)

add_executable(TestRos TestRos.cpp)
target_link_libraries(TestRos ${TestLibs} dl)
add_dependencies(TestRos ${GTEST_DEPENDS} CartesianInterface )
add_test(NAME TestRos COMMAND TestRos)

