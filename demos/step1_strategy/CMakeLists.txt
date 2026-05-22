add_executable(demo_step1_strategy main.cpp)
target_link_libraries(demo_step1_strategy
    PRIVATE TinyCell::core TinyCell::io TinyCell::solver
)
target_compile_definitions(demo_step1_strategy PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
