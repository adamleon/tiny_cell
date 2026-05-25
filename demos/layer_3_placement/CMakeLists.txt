add_executable(demo_layer_3_placement main.cpp)
target_link_libraries(demo_layer_3_placement PRIVATE
    TinyCell::core
    TinyCell::io
    TinyCell::solver
    TinyCell::svg
)
target_compile_definitions(demo_layer_3_placement PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
    TINYCELL_DEMO_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/output"
)
