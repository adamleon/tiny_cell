# svg/ — stdlib-only debug-oracle output. No foreign-lib includes.
# Reads core types directly (CLAUDE.md §0 svg/ row).

add_library(tinycell_svg STATIC
    src/svg.cpp
    src/workflow_svg.cpp
)
target_include_directories(tinycell_svg PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(tinycell_svg
    PUBLIC TinyCell::core
)
target_compile_features(tinycell_svg PUBLIC cxx_std_20)
add_library(TinyCell::svg ALIAS tinycell_svg)
