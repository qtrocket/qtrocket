
add_executable(model_tests
   PartTests.cpp
)

target_link_libraries(model_tests PRIVATE
   model
   utils
   GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(model_tests)

add_test(NAME qtrocket_model_tests COMMAND model_tests)

