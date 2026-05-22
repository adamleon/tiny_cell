add_library(tinycell_core INTERFACE)
target_include_directories(tinycell_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(tinycell_core INTERFACE
    mp-units::mp-units
)
target_compile_features(tinycell_core INTERFACE cxx_std_20)
add_library(TinyCell::core ALIAS tinycell_core)
