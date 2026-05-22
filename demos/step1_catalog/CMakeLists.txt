add_executable(demo_step1_catalog main.cpp)
target_link_libraries(demo_step1_catalog PRIVATE TinyCell::core TinyCell::io)
target_compile_definitions(demo_step1_catalog PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
