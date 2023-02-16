include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.11.0
)

FetchContent_MakeAvailable(googletest)
add_library(GTest::GTest INTERFACE IMPORTED)
target_link_libraries(GTest::GTest INTERFACE gtest_main)

add_executable(usstandardatmosphere_test USStandardAtmosphereTests.cpp)
target_link_libraries(usstandardatmosphere_test
   PRIVATE
   GTest::GTest
   sim)

add_test(sim_tests usstandardatmosphere_test)