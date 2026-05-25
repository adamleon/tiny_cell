cmake_minimum_required(VERSION 3.28)

# Intent: build two tiny executables, each solving the same trivial NLP
# (min x²+y² s.t. x+y>=5) through a candidate library. Success is "both
# binaries run and converge to (2.5, 2.5) within tolerance"; if either
# library doesn't build under vcpkg on Windows, the failure is the
# finding — record it in FINDINGS.md and move on.

project(tinycell-layer3-spike LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- IPOPT (vcpkg port: coin-or-ipopt) ---
# Ships pkg-config only, no CMake config. Use find_library + find_path.
# Headers live under include/coin-or/Ip*.hpp.
find_library(IPOPT_LIBRARY NAMES ipopt)
find_path(IPOPT_INCLUDE_DIR coin-or/IpIpoptApplication.hpp)

if(IPOPT_LIBRARY AND IPOPT_INCLUDE_DIR)
    add_executable(ipopt_smoke src/ipopt_smoke.cpp)
    # Add include/coin-or so the smoke test's #include <IpXxx.hpp> works
    # without the coin-or/ prefix (matches IPOPT-upstream convention).
    target_include_directories(ipopt_smoke PRIVATE "${IPOPT_INCLUDE_DIR}/coin-or")
    target_link_libraries(ipopt_smoke PRIVATE "${IPOPT_LIBRARY}")
    message(STATUS "Spike: IPOPT found at ${IPOPT_LIBRARY}")
else()
    message(WARNING "Spike: IPOPT not located (lib=${IPOPT_LIBRARY}, inc=${IPOPT_INCLUDE_DIR}) — skipping ipopt_smoke")
endif()

# --- NLopt (vcpkg port: nlopt) ---
# Ships a proper CMake config. Clean.
find_package(NLopt CONFIG QUIET)
if(NLopt_FOUND)
    add_executable(nlopt_smoke src/nlopt_smoke.cpp)
    target_link_libraries(nlopt_smoke PRIVATE NLopt::nlopt)
    message(STATUS "Spike: NLopt found — nlopt_smoke will build")
else()
    message(WARNING "Spike: NLopt not found via find_package(NLopt CONFIG) — skipping nlopt_smoke")
endif()

# --- Runtime DLL deployment ---
# vcpkg's CMake integration normally copies dependent DLLs next to the
# .exe on Windows automatically (X_VCPKG_APPLOCAL_DEPS_INSTALL hooks).
# If a smoke run fails with "ipopt-3.dll not found" we'll wire it
# explicitly here.
