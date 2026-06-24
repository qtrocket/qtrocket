# End-to-end tests that drive the real RocketModel / Propagator / Environment and the
# motor database via the QtRocket controller, which is linked in from the
# qtrocket_core static lib (as for the executables).
add_executable(integration_tests
   PhysicsIntegrationTests.cpp
   MotorDatabasePersistenceTests.cpp
   DesignPersistenceTests.cpp
   PlacementInvarianceTests.cpp
   PokeThroughRegressionTests.cpp
)
qtrocket_enable_coverage(integration_tests)

# Absolute paths so the tests run from any cwd: the bundled motor database, and the design-fixture
# corpus + the committed Part-Placement invariance baseline under tests/data.
target_compile_definitions(integration_tests PRIVATE
   QTROCKET_DATA_DIR="${CMAKE_SOURCE_DIR}/data"
   QTROCKET_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/tests/data")

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

# Comprehensive CLI design / persistence / concrete-part-type matrix tests. Drive the real Repl across
# the committed design fixtures (tests/data/designs/*.qrd) and motor fixtures (tests/data/motors/*.qmd
# + the bundled data/Aerotech.rse), exercising round-trip fidelity, geometry/part-type invariants, and
# a flight matrix spanning impulse classes 1/4A -> M across multiple motor diameters.
add_executable(design_matrix_tests
   DesignMatrixTests.cpp
)
qtrocket_enable_coverage(design_matrix_tests)

target_compile_definitions(design_matrix_tests PRIVATE
   QTROCKET_DATA_DIR="${CMAKE_SOURCE_DIR}/data"
   QTROCKET_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/tests/data")

target_link_libraries(design_matrix_tests
   cli
   GTest::gtest_main
)

gtest_discover_tests(design_matrix_tests)

# The ladder-sweep cases (FlightMatrix, Atmosphere) fly every motor of each class's full 1/4A -> M
# ladder, in vacuum and through the atmosphere -- with high-impulse motors coasting to tens of km this
# dominates the runtime. Register the suite two ways:
#   * the default ("light") entry runs the whole suite but flies only ONE motor per class for the
#     sweeps (a fast smoke check that still covers every diameter);
#   * the "heavy" entry re-runs just the sweeps with QTROCKET_FULL_LADDER=1, flying the COMPLETE ladder
#     (and so the apogee-monotonicity and per-motor drag checks). It is LABELS-tagged `heavy`.
# So: `ctest -R qtrocket_*` runs both (full coverage); `ctest -LE heavy` is the fast loop (everything
# except the full-ladder sweeps, including the unit suites); `ctest -L heavy` runs only the sweeps.
add_test(NAME qtrocket_design_matrix_tests COMMAND design_matrix_tests)

add_test(NAME qtrocket_design_matrix_heavy_tests
         COMMAND design_matrix_tests "--gtest_filter=FlightMatrix.*:Atmosphere.*")
set_tests_properties(qtrocket_design_matrix_heavy_tests PROPERTIES
   LABELS heavy
   ENVIRONMENT "QTROCKET_FULL_LADDER=1")
