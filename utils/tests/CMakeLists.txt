
add_executable(utils_tests
   ThrustCurveParserTests.cpp
)

target_link_libraries(utils_tests
   utils
   GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(utils_tests)

add_test(NAME qtrocket_utils_tests COMMAND utils_tests)
