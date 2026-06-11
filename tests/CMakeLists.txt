# End-to-end tests that drive the real RocketModel / Propagator / Environment and the
# motor database via the QtRocket controller, which is linked in from the
# qtrocket_core static lib (as for the executables).
add_executable(integration_tests
   PhysicsIntegrationTests.cpp
   MotorDatabasePersistenceTests.cpp
)

# Absolute path to the bundled motor database, so the test runs from any cwd.
target_compile_definitions(integration_tests PRIVATE
   QTROCKET_DATA_DIR="${CMAKE_SOURCE_DIR}/data")

target_link_libraries(integration_tests
   qtrocket_core
   GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(integration_tests)

add_test(NAME qtrocket_integration_tests COMMAND integration_tests)
