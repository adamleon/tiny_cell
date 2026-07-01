add_executable(demo_positional_prior main.cpp)
target_link_libraries(demo_positional_prior PRIVATE
    TinyCell::core
    TinyCell::solver
    TinyCell::svg
)
target_compile_definitions(demo_positional_prior PRIVATE
    TINYCELL_DEMO_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/output"
)
