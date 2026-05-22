add_executable(demo_brute_force_enumerator main.cpp)
target_link_libraries(demo_brute_force_enumerator
    PRIVATE TinyCell::core TinyCell::io TinyCell::solver
)
target_compile_definitions(demo_brute_force_enumerator PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
