
add_executable(sim_tests
   USStandardAtmosphereTests.cpp
   SimpleAtmosphereTests.cpp
   EnvironmentTests.cpp
   RK45SolverTests.cpp
   SphericalGeoidAndGravityTests.cpp
   IntegratorTests.cpp
)
qtrocket_enable_coverage(sim_tests)

target_link_libraries(sim_tests
   sim
   GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(sim_tests)

add_test(NAME qtrocket_sim_tests COMMAND sim_tests)
