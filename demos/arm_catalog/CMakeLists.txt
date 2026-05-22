add_executable(demo_arm_catalog main.cpp)
target_link_libraries(demo_arm_catalog PRIVATE TinyCell::core TinyCell::io)
target_compile_definitions(demo_arm_catalog PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
