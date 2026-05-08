cmake_minimum_required(VERSION 3.20)
project(TinyCell VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

option(TINYCELL_BUILD_DEMOS "Build demo executables (requires OpenGL + X11)" ON)

set(THREEPP_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(THREEPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(THREEPP_WITH_WGPU      OFF CACHE BOOL "" FORCE)
set(THREEPP_WITH_VULKAN    OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    threepp
    GIT_REPOSITORY https://github.com/markaren/threepp.git
    GIT_TAG 64808748c76ef02948034544372e5fc8e855c09d
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

# ── Warning flags (compiler-portable) ─────────────────────────────────────────
if(MSVC)
    set(TINYCELL_WARNINGS         /W4)
    set(TINYCELL_WARNINGS_WERROR  /W4 /WX)
else()
    set(TINYCELL_WARNINGS         -Wall -Wextra)
    set(TINYCELL_WARNINGS_WERROR  ${TINYCELL_WARNINGS_WERROR})
endif()

# ── Demo helper ───────────────────────────────────────────────────────────────
function(add_demo name)
    add_executable(${name} src/demos/${name}.cpp)
    target_link_libraries(${name} PRIVATE threepp::threepp EnTT::EnTT glm::glm)
    target_include_directories(${name} PRIVATE
        src
        ${threepp_SOURCE_DIR}/src/external/nlohmann
    )
    target_compile_options(${name} PRIVATE ${TINYCELL_WARNINGS})
endfunction()

# ── Demos ─────────────────────────────────────────────────────────────────────
if(TINYCELL_BUILD_DEMOS)
    add_demo(fence_walls)
    add_demo(palletizing)
endif()

# ── Generator tool (no threepp, just nlohmann/json) ───────────────────────────
add_executable(GenLookup src/tools/gen_lookup.cpp)
target_include_directories(GenLookup PRIVATE
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_compile_options(GenLookup PRIVATE ${TINYCELL_WARNINGS})

# ── Unit tests (no threepp; data model + solver only) ────────────────────────
enable_testing()

add_executable(TestCell tests/test_cell.cpp)
target_include_directories(TestCell PRIVATE
    src
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_compile_options(TestCell PRIVATE ${TINYCELL_WARNINGS_WERROR})

add_test(NAME cell_tests
    COMMAND TestCell
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_executable(TestSolver tests/test_solver.cpp)
target_include_directories(TestSolver PRIVATE
    src
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_compile_options(TestSolver PRIVATE ${TINYCELL_WARNINGS_WERROR})

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
target_compile_options(TestFactoryScene PRIVATE ${TINYCELL_WARNINGS_WERROR})

add_test(NAME factory_scene_tests
    COMMAND TestFactoryScene
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_executable(TestSim tests/test_sim.cpp)
target_link_libraries(TestSim PRIVATE EnTT::EnTT glm::glm)
target_include_directories(TestSim PRIVATE
    src
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_compile_options(TestSim PRIVATE ${TINYCELL_WARNINGS_WERROR})

add_test(NAME sim_tests
    COMMAND TestSim
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
