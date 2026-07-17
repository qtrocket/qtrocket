find_package(Qt6 REQUIRED COMPONENTS Core Widgets Test)

# Qt-side model tests: RocketPartModel's QAbstractItemModel contract under the granular event
# stream (QAbstractItemModelTester). Widget-free at runtime -- QCoreApplication, no platform plugin.
add_executable(gui_model_tests
   RocketPartModelTests.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/../RocketTreeView.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/../RocketTreeView.h
)
qtrocket_enable_coverage(gui_model_tests)

target_include_directories(gui_model_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)

target_link_libraries(gui_model_tests PRIVATE
   Qt6::Widgets
   Qt6::Test
   qtrocket_core
   GTest::gtest
)

include(GoogleTest)
gtest_discover_tests(gui_model_tests)

add_test(NAME qtrocket_gui_model_tests COMMAND gui_model_tests)
