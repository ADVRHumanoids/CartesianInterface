

add_executable(cartesio_solver cartesio_solver.cpp)
target_compile_definitions(cartesio_solver PRIVATE
    -DURDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../urdf/coman.urdf"
    -DSRDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../srdf/coman.srdf"
    -DIK_PB_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../configs/coman_stack.yaml")
target_link_libraries(cartesio_solver CartesianInterface)
