add_executable(demo_solve_workflow main.cpp)
target_link_libraries(demo_solve_workflow PRIVATE
    TinyCell::core
    TinyCell::io
    TinyCell::solver
    TinyCell::boost_conv
    TinyCell::svg
)
target_compile_definitions(demo_solve_workflow PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
    TINYCELL_DEMO_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/output"
)
