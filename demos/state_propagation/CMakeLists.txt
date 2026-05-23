add_executable(demo_state_propagation main.cpp)
target_link_libraries(demo_state_propagation
    PRIVATE TinyCell::core TinyCell::io TinyCell::solver
)
target_compile_definitions(demo_state_propagation PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
