add_executable(demo_catalog_svg main.cpp)
target_link_libraries(demo_catalog_svg PRIVATE
    TinyCell::core
    TinyCell::io
    TinyCell::boost_conv
    TinyCell::svg
)
target_compile_definitions(demo_catalog_svg PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
    TINYCELL_DEMO_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/output"
)
