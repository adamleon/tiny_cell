cmake_minimum_required(VERSION 3.20)
project(TinyCell VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

set(THREEPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(THREEPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    threepp
    GIT_REPOSITORY https://github.com/markaren/threepp.git
    GIT_TAG 57aae69f08deba6e361dc8ec86c4ca6841fac43c
)
FetchContent_MakeAvailable(threepp)

# ── Demo helper ───────────────────────────────────────────────────────────────
function(add_demo name)
    add_executable(${name} src/demos/${name}.cpp)
    target_link_libraries(${name} PRIVATE threepp::threepp)
    target_include_directories(${name} PRIVATE
        src
        ${threepp_SOURCE_DIR}/src/external/nlohmann
    )
    target_compile_options(${name} PRIVATE -Wall -Wextra)
endfunction()

# ── Demos ─────────────────────────────────────────────────────────────────────
add_demo(fence_walls)

# ── Generator tool (no threepp, just nlohmann/json) ───────────────────────────
add_executable(GenLookup src/tools/gen_lookup.cpp)
target_include_directories(GenLookup PRIVATE
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_compile_options(GenLookup PRIVATE -Wall -Wextra)
