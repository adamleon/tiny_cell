add_executable(demo_transport_workflow main.cpp)
target_link_libraries(demo_transport_workflow PRIVATE
    TinyCell::core
    TinyCell::io
    TinyCell::solver
)
target_compile_definitions(demo_transport_workflow PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
