add_executable(test_smoke test_smoke.cpp)
target_link_libraries(test_smoke PRIVATE
    TinyCell::core
    GTest::gtest
    GTest::gtest_main
)
add_test(NAME test_smoke COMMAND test_smoke)
