add_library(tinycell_solver STATIC
    src/allocator.cpp
    src/anchor_strategy.cpp
    src/arm_strategy.cpp
    src/cost.cpp
    src/pusher_strategy.cpp
    src/enumerator.cpp
    src/motion_model.cpp
    src/state_propagation.cpp
    src/transfer_strategy.cpp
)
target_include_directories(tinycell_solver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(tinycell_solver
    PUBLIC TinyCell::core
)
target_compile_features(tinycell_solver PUBLIC cxx_std_20)
add_library(TinyCell::solver ALIAS tinycell_solver)
