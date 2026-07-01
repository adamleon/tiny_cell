# adapters/boost_conv — the ONLY home of boost/* includes (CLAUDE.md §0).
# Public API exposes only core types; Boost is a PRIVATE link.

find_package(boost_geometry REQUIRED CONFIG)

add_library(tinycell_boost_conv STATIC
    src/boost_conv.cpp
)
target_include_directories(tinycell_boost_conv PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(tinycell_boost_conv
    PUBLIC TinyCell::core
    PRIVATE Boost::geometry
)
target_compile_features(tinycell_boost_conv PUBLIC cxx_std_20)
add_library(TinyCell::boost_conv ALIAS tinycell_boost_conv)
