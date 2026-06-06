# End-to-end physics integration tests. These drive the real RocketModel /
# Propagator / Environment via the QtRocket controller, so QtRocket.cpp is
# compiled in directly (as for the executables) to avoid a utils<->controller
# circular static-lib dependency. No Qt is needed (the controller is Qt-free).
add_executable(integration_tests
   PhysicsIntegrationTests.cpp
   ${CMAKE_SOURCE_DIR}/QtRocket.cpp
)

# Absolute path to the bundled motor database, so the test runs from any cwd.
target_compile_definitions(integration_tests PRIVATE
   QTROCKET_DATA_DIR="${CMAKE_SOURCE_DIR}/data")

target_link_libraries(integration_tests
   utils
   sim
   model
   GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(integration_tests)

add_test(NAME qtrocket_integration_tests COMMAND integration_tests)
