add_executable(demo_arm_strategy main.cpp)
target_link_libraries(demo_arm_strategy
    PRIVATE TinyCell::core TinyCell::io TinyCell::solver
)
target_compile_definitions(demo_arm_strategy PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
