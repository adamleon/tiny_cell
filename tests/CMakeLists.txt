add_executable(test_smoke test_smoke.cpp)
target_link_libraries(test_smoke PRIVATE
    TinyCell::core
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_smoke COMMAND test_smoke)

add_executable(test_catalog test_catalog.cpp)
target_link_libraries(test_catalog PRIVATE
    TinyCell::core
    TinyCell::io
    GTest::gtest
    GTest::gtest_main
)
target_compile_definitions(test_catalog PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
add_test(NAME test_catalog COMMAND test_catalog)

add_executable(test_workflow_loader test_workflow_loader.cpp)
target_link_libraries(test_workflow_loader PRIVATE
    TinyCell::core
    TinyCell::io
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_workflow_loader COMMAND test_workflow_loader)

add_executable(test_workflow_svg test_workflow_svg.cpp)
target_link_libraries(test_workflow_svg PRIVATE
    TinyCell::core
    TinyCell::io
    TinyCell::svg
    GTest::gtest
    GTest::gtest_main
)
target_compile_definitions(test_workflow_svg PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
add_test(NAME test_workflow_svg COMMAND test_workflow_svg)

add_executable(test_demo_scenario test_demo_scenario.cpp)
target_link_libraries(test_demo_scenario PRIVATE
    TinyCell::core
    TinyCell::io
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
target_compile_definitions(test_demo_scenario PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
add_test(NAME test_demo_scenario COMMAND test_demo_scenario)

add_executable(test_strategy test_strategy.cpp)
target_link_libraries(test_strategy PRIVATE
    TinyCell::core
    TinyCell::io
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
target_compile_definitions(test_strategy PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
add_test(NAME test_strategy COMMAND test_strategy)

add_executable(test_enumerator test_enumerator.cpp)
target_link_libraries(test_enumerator PRIVATE
    TinyCell::core
    TinyCell::io
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
target_compile_definitions(test_enumerator PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
add_test(NAME test_enumerator COMMAND test_enumerator)

add_executable(test_state_propagation test_state_propagation.cpp)
target_link_libraries(test_state_propagation PRIVATE
    TinyCell::core
    TinyCell::io
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
target_compile_definitions(test_state_propagation PRIVATE
    TINYCELL_REPO_ROOT="${CMAKE_SOURCE_DIR}"
)
add_test(NAME test_state_propagation COMMAND test_state_propagation)

add_executable(test_allocator test_allocator.cpp)
target_link_libraries(test_allocator PRIVATE
    TinyCell::core
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_allocator COMMAND test_allocator)

add_executable(test_cost test_cost.cpp)
target_link_libraries(test_cost PRIVATE
    TinyCell::core
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_cost COMMAND test_cost)

add_executable(test_ports test_ports.cpp)
target_link_libraries(test_ports PRIVATE
    TinyCell::core
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_ports COMMAND test_ports)

add_executable(test_transfer_strategy test_transfer_strategy.cpp)
target_link_libraries(test_transfer_strategy PRIVATE
    TinyCell::core
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_transfer_strategy COMMAND test_transfer_strategy)

add_executable(test_anchor_strategy test_anchor_strategy.cpp)
target_link_libraries(test_anchor_strategy PRIVATE
    TinyCell::core
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_anchor_strategy COMMAND test_anchor_strategy)

add_executable(test_boost_conv test_boost_conv.cpp)
target_link_libraries(test_boost_conv PRIVATE
    TinyCell::core
    TinyCell::boost_conv
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_boost_conv COMMAND test_boost_conv)

add_executable(test_svg test_svg.cpp)
target_link_libraries(test_svg PRIVATE
    TinyCell::core
    TinyCell::svg
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_svg COMMAND test_svg)

add_executable(test_frames test_frames.cpp)
target_link_libraries(test_frames PRIVATE
    TinyCell::core
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_frames COMMAND test_frames)

add_executable(test_positional_prior test_positional_prior.cpp)
target_link_libraries(test_positional_prior PRIVATE
    TinyCell::core
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_positional_prior COMMAND test_positional_prior)

add_executable(test_layout_objective test_layout_objective.cpp)
target_link_libraries(test_layout_objective PRIVATE
    TinyCell::core
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_layout_objective COMMAND test_layout_objective)

add_executable(test_layout_solve test_layout_solve.cpp)
target_link_libraries(test_layout_solve PRIVATE
    TinyCell::core
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_layout_solve COMMAND test_layout_solve)

add_executable(test_belt_routing test_belt_routing.cpp)
target_link_libraries(test_belt_routing PRIVATE
    TinyCell::core
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_belt_routing COMMAND test_belt_routing)

add_executable(test_workflow_validation test_workflow_validation.cpp)
target_link_libraries(test_workflow_validation PRIVATE
    TinyCell::core
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_workflow_validation COMMAND test_workflow_validation)

add_executable(test_station_template test_station_template.cpp)
target_link_libraries(test_station_template PRIVATE
    TinyCell::core
    TinyCell::solver
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_station_template COMMAND test_station_template)
