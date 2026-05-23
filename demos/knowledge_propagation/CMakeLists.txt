add_executable(demo_knowledge_propagation main.cpp)
target_link_libraries(demo_knowledge_propagation
    PRIVATE TinyCell::core TinyCell::io TinyCell::solver
)
target_compile_definitions(demo_knowledge_propagation PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
