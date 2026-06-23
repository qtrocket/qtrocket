
add_executable(model_tests
   PartTests.cpp
   MotorTests.cpp
   ThrustCurveTests.cpp
   ThrustCurveParserTests.cpp
   RASPLoaderTests.cpp
   InertiaTensorsTests.cpp
   BodyTubeTests.cpp
   NoseConeTests.cpp
   FinSetTests.cpp
   AeroTests.cpp
   PartFactoryTests.cpp
   RocketModelTests.cpp
   PlacementTypesTests.cpp
   GeometryProfileTests.cpp
   ResolverSweepTests.cpp
)
qtrocket_enable_coverage(model_tests)

target_link_libraries(model_tests PRIVATE
   model
   utils
   GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(model_tests)

add_test(NAME qtrocket_model_tests COMMAND model_tests)
