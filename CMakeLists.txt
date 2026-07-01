cmake_minimum_required(VERSION 3.28)

project(TinyCell
    VERSION 0.1.0
    DESCRIPTION "Generative design of robotic cells"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(TINYCELL_BUILD_TESTS "Build tests" ON)
option(TINYCELL_BUILD_DEMOS "Build per-milestone demos" ON)
# Post-MVP sales-demo track (HANDOFF.md). OFF by default so the core build/tests
# are byte-for-byte unaffected; ON pulls threepp + Vulkan into render/ only.
option(TINYCELL_WITH_VULKAN "Build the post-MVP Vulkan render module (requires the Vulkan SDK + threepp)" OFF)

include(FetchContent)

# gsl-lite — contract-checking backend used by mp-units (engineering.md §3.2:
# mp-units contract checking ON in both debug and release).
FetchContent_Declare(
    gsl-lite
    GIT_REPOSITORY https://github.com/gsl-lite/gsl-lite.git
    GIT_TAG v0.41.0
)
FetchContent_MakeAvailable(gsl-lite)

# mp-units — dimensional safety in core/. Pinned to a v2 release; bumping is deliberate
# (engineering.md §2.3 — pin under version sensitivity, v3 breaking on the horizon).
FetchContent_Declare(
    mp-units
    GIT_REPOSITORY https://github.com/mpusz/mp-units.git
    GIT_TAG v2.4.0
    SOURCE_SUBDIR src
)
set(MP_UNITS_BUILD_CXX_MODULES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(mp-units)

# nlohmann/json — JSON parsing in io/. Catalog & workflow input format.
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nlohmann_json)

add_subdirectory(core)
add_subdirectory(io)
add_subdirectory(adapters/boost_conv)
add_subdirectory(svg)
add_subdirectory(solver)

if(TINYCELL_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    enable_testing()
    add_subdirectory(tests)
endif()

if(TINYCELL_BUILD_DEMOS)
    add_subdirectory(demos)
endif()

# ── Post-MVP sales-demo track: interactive 3D via threepp (Vulkan) ───────────
# Opt-in and OFF by default (see the option above) so the core build/tests are
# untouched. threepp + Vulkan are foreign libs confined to render/ (CLAUDE.md
# §0/§1). threepp has no vcpkg port, so it enters via FetchContent like the other
# source deps above. See HANDOFF.md for the demo plan.
if(TINYCELL_WITH_VULKAN)
    # threepp's VulkanRenderer #includes <vk_mem_alloc.h> (Vulkan Memory Allocator),
    # which the LunarG SDK does not bundle. Fetch the header and pre-seed threepp's
    # VMA search (find_path(VMA_INCLUDE_DIR ...)) BEFORE threepp is configured.
    FetchContent_Declare(
        vma
        GIT_REPOSITORY https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator.git
        GIT_TAG v3.1.0
    )
    FetchContent_GetProperties(vma)
    if(NOT vma_POPULATED)
        FetchContent_Populate(vma)   # header-only: download, don't add its CMake
    endif()
    set(VMA_INCLUDE_DIR "${vma_SOURCE_DIR}/include" CACHE PATH "VMA include dir for threepp" FORCE)

    set(THREEPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(THREEPP_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(THREEPP_WITH_VULKAN    ON  CACHE BOOL "" FORCE)
    FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG master   # research-grade + fast-moving: pin a commit before the demo ships
        SYSTEM
    )
    FetchContent_MakeAvailable(threepp)

    find_package(Vulkan REQUIRED)
    message(STATUS "TinyCell: Vulkan render module ON (Vulkan ${Vulkan_VERSION})")

    add_subdirectory(render)
endif()
