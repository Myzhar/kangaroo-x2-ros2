include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

add_executable(example_mixed_model example_mixed_model.cpp)
target_link_libraries(example_mixed_model kangaroo_x2_lib)
