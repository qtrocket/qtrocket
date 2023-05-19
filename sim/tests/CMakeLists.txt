enable_testing()

add_executable(sim_tests
   USStandardAtmosphereTests.cpp
)

target_link_libraries(sim_tests
   sim
   GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(sim_tests)

