# pipeline/ — application-level orchestration over solver + adapters + core.
# Builds a LayoutProblem (+ render/debug side tables) from a solved allocation.
# NOT part of solver/ because it uses the boost adapter (CLAUDE.md §1). Shared
# by the SVG demo and the threepp render app so both build the identical layout.

add_library(tinycell_pipeline STATIC
    src/layout_builder.cpp
)
target_include_directories(tinycell_pipeline PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(tinycell_pipeline
    PUBLIC  TinyCell::core TinyCell::solver
    PRIVATE TinyCell::boost_conv        # convex_hull / buffer_outward; no boost type leaks out
)
target_compile_features(tinycell_pipeline PUBLIC cxx_std_20)
add_library(TinyCell::pipeline ALIAS tinycell_pipeline)
