# End-to-end tests that drive the real RocketModel / Propagator / Environment and the
# motor database via the QtRocket controller, which is linked in from the
# qtrocket_core static lib (as for the executables).
add_executable(integration_tests
   PhysicsIntegrationTests.cpp
   MotorDatabasePersistenceTests.cpp
   DesignPersistenceTests.cpp
)
qtrocket_enable_coverage(integration_tests)

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

# Unit tests for the Propagator's hang-proof termination guards and trajectory statistics,
# driven by a mock Propagatable (no motor DB, no Qt). Links model, which transitively pulls in
# sim + utils and provides the out-of-line model::Propagatable constructor symbol.
add_executable(propagator_tests
   PropagatorTests.cpp
)
qtrocket_enable_coverage(propagator_tests)

target_link_libraries(propagator_tests
   model
   GTest::gtest_main
)

gtest_discover_tests(propagator_tests)

add_test(NAME qtrocket_propagator_tests COMMAND propagator_tests)

# End-to-end CLI/REPL design-command tests: drive the real Repl (from the cli lib) through a
# scripted build -> save -> reload -> fly session.
add_executable(cli_tests
   CliDesignCommandsTests.cpp
)
qtrocket_enable_coverage(cli_tests)

target_compile_definitions(cli_tests PRIVATE
   QTROCKET_DATA_DIR="${CMAKE_SOURCE_DIR}/data")

target_link_libraries(cli_tests
   cli
   GTest::gtest_main
)

gtest_discover_tests(cli_tests)

add_test(NAME qtrocket_cli_tests COMMAND cli_tests)
