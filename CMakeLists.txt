cmake_minimum_required(VERSION 3.20)
project(TinyCell VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

set(THREEPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(THREEPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(THREEPP_WITH_WGPU ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    threepp
    GIT_REPOSITORY https://github.com/markaren/threepp.git
    GIT_TAG 1d3a6ab58f9aa392e7d8869cd5671b3c62c5d27a
)
FetchContent_MakeAvailable(threepp)

FetchContent_Declare(
    entt
    GIT_REPOSITORY https://github.com/skypjack/entt.git
    GIT_TAG v3.16.0
)
FetchContent_MakeAvailable(entt)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.3
)
FetchContent_MakeAvailable(glm)

# ── Demo helper ───────────────────────────────────────────────────────────────
function(add_demo name)
    add_executable(${name} src/demos/${name}.cpp)
    target_link_libraries(${name} PRIVATE threepp::threepp EnTT::EnTT glm::glm)
    target_include_directories(${name} PRIVATE
        src
        ${threepp_SOURCE_DIR}/src/external/nlohmann
    )
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    if(WIN32 AND WGPU_NATIVE_DLL)
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${WGPU_NATIVE_DLL}"
                "$<TARGET_FILE_DIR:${name}>/wgpu_native.dll"
            COMMENT "Copying wgpu_native.dll to ${name} output directory"
        )
    endif()
endfunction()

# ── Demos ─────────────────────────────────────────────────────────────────────
add_demo(fence_walls)
add_demo(palletizing)

# ── Generator tool (no threepp, just nlohmann/json) ───────────────────────────
add_executable(GenLookup src/tools/gen_lookup.cpp)
target_include_directories(GenLookup PRIVATE
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_compile_options(GenLookup PRIVATE -Wall -Wextra)

# ── Unit tests (no threepp; data model + solver only) ────────────────────────
enable_testing()

add_executable(TestCell tests/test_cell.cpp)
target_include_directories(TestCell PRIVATE
    src
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_compile_options(TestCell PRIVATE -Wall -Wextra)

add_test(NAME cell_tests
    COMMAND TestCell
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_executable(TestSolver tests/test_solver.cpp)
target_include_directories(TestSolver PRIVATE
    src
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_compile_options(TestSolver PRIVATE -Wall -Wextra)

add_test(NAME solver_tests
    COMMAND TestSolver
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_executable(TestFactoryScene tests/test_factory_scene.cpp)
target_link_libraries(TestFactoryScene PRIVATE EnTT::EnTT glm::glm)
target_include_directories(TestFactoryScene PRIVATE
    src
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_compile_options(TestFactoryScene PRIVATE -Wall -Wextra)

add_test(NAME factory_scene_tests
    COMMAND TestFactoryScene
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
