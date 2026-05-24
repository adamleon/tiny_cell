add_executable(demo_layer_2_allocation main.cpp)
target_link_libraries(demo_layer_2_allocation
    PRIVATE TinyCell::core TinyCell::io TinyCell::solver
)
target_compile_definitions(demo_layer_2_allocation PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
