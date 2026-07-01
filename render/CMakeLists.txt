# render/ — interactive 3D via threepp's Vulkan backend (post-MVP sales-demo track).
# Foreign libs (threepp, Vulkan) are confined here per CLAUDE.md §0/§1. This
# directory is only added when TINYCELL_WITH_VULKAN=ON (the root add_subdirectory
# is gated on it), so the core build stays free of any graphics dependency.

# Phase-0 spike: prove threepp's Vulkan deferred-hybrid backend builds + runs in
# this tree. NOT the real render library yet — that grows here in Phase 2, reading
# a solver LayoutSolution. The spike carries no tiny_cell types by design.
add_executable(tinycell_render_spike src/spike.cpp)
target_compile_features(tinycell_render_spike PRIVATE cxx_std_20)
target_link_libraries(tinycell_render_spike PRIVATE threepp)

# The spike's TU includes VulkanRenderer.hpp; threepp gates its Vulkan-only
# declarations behind this macro (matches how threepp builds its own Vulkan TUs).
target_compile_definitions(tinycell_render_spike PRIVATE THREEPP_WITH_VULKAN=1)

# threepp links STATIC here and Vulkan's vulkan-1.dll is a system DLL, so no
# runtime DLLs need copying next to the exe. (If threepp is ever built SHARED,
# add a $<TARGET_RUNTIME_DLLS:...> copy guarded so it's skipped when empty.)
