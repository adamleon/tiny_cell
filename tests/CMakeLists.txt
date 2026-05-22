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
