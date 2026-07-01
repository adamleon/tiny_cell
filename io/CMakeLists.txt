add_library(tinycell_io STATIC
    src/catalog_loader.cpp
    src/workflow_loader.cpp
)
target_include_directories(tinycell_io PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(tinycell_io
    PUBLIC TinyCell::core
    PRIVATE nlohmann_json::nlohmann_json
)
target_compile_features(tinycell_io PUBLIC cxx_std_20)
add_library(TinyCell::io ALIAS tinycell_io)
