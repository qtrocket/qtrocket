cmake_minimum_required(VERSION 3.25)

project(qtrocket VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(QTROCKET_ENABLE_COVERAGE "Enable LLVM source-based coverage for project/test targets" OFF)


include(FetchContent)

# Google Test framework
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest
    GIT_TAG        v1.17.0)
if(WIN32)
   set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
endif()
FetchContent_MakeAvailable(googletest)

# fmtlib dependency
#FetchContent_Declare(fmt
#    GIT_REPOSITORY https://github.com/fmtlib/fmt
#    GIT_TAG        9.1.0)
#FetchContent_MakeAvailable(fmt)

# jsoncpp dependency
FetchContent_Declare(jsoncpp
    GIT_REPOSITORY https://github.com/open-source-parsers/jsoncpp
    GIT_TAG        1.9.7)
set(JSONCPP_WITH_TESTS OFF)
set(JSONCPP_WITH_EXAMPLE OFF)
set(JSONCPP_WITH_PKGCONFIG_SUPPORT OFF)
set(JSONCPP_BUILD_SHARED_LIBS OFF)
set(JSONCPP_BUILD_OBJECT_LIBS OFF)
set(JSONCPP_BUILD_STATIC_LIBS ON)
FetchContent_MakeAvailable(jsoncpp)

# curl dependency
FetchContent_Declare(CURL
    GIT_REPOSITORY https://github.com/curl/curl
    GIT_TAG        curl-8_20_0)
set(BUILD_CURL_EXE OFF)
set(BUILD_SHARED_LIBS OFF)
set(HTTP_ONLY ON)
set(SSL_ENABLED ON)
set(CURL_USE_LIBPSL OFF)
set(BUILD_TESTING OFF)
# Disable curl's optional third-party dependencies. We only use HTTPS GET, 
# so none of these are needed. Several default to ON/AUTO, which makes
# the build non-hermetic: curl links whatever it auto-detects on the build host
# (e.g. Homebrew libssh2/nghttp2/brotli/zstd on the macOS CI runner) and the
# final link then fails because those libs aren't on the linker search path
# Forcing them off keeps the build identical on every platform.
set(CURL_USE_LIBSSH2 OFF)
set(CURL_USE_LIBSSH OFF)
set(USE_NGHTTP2 OFF)
set(USE_LIBIDN2 OFF)
# The BROTLI and ZSTD are to get Github CI actions to work on macos-latest.
set(CURL_BROTLI OFF CACHE STRING "" FORCE)
set(CURL_ZSTD OFF CACHE STRING "" FORCE)
if(WIN32)
   set(CURL_USE_SCHANNEL ON)
endif()
FetchContent_MakeAvailable(CURL)

# eigen dependency
FetchContent_Declare(Eigen
   GIT_REPOSITORY https://gitlab.com/libeigen/eigen
   GIT_TAG 5.0.1)
FetchContent_MakeAvailable(Eigen)

# boost dependency

FetchContent_Declare(Boost
   GIT_REPOSITORY https://github.com/boostorg/boost
   GIT_TAG boost-1.91.0-1)
set(BOOST_INCLUDE_LIBRARIES property_tree)
FetchContent_MakeAvailable(Boost)
# Add qtrocket subdirectories. These are components that will be linked in 


enable_testing()
if (MSVC)
    # warning level 4
    add_compile_options(/W4 /WX)
else()
    # additional warnings
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

if(QTROCKET_ENABLE_COVERAGE)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "QTROCKET_ENABLE_COVERAGE uses llvm-cov and requires Clang/AppleClang. Use the coverage-clang preset, or disable coverage for GCC/MSVC builds.")
    endif()

    find_program(LLVM_COV_EXECUTABLE NAMES llvm-cov llvm-cov-21 llvm-cov-20 llvm-cov-19 llvm-cov-18 llvm-cov-17 REQUIRED)
    find_program(LLVM_PROFDATA_EXECUTABLE NAMES llvm-profdata llvm-profdata-21 llvm-profdata-20 llvm-profdata-19 llvm-profdata-18 llvm-profdata-17 REQUIRED)

    set(QTROCKET_COVERAGE_COMPILE_OPTIONS
        -O0
        -g
        -fprofile-instr-generate
        -fcoverage-mapping)
    set(QTROCKET_COVERAGE_LINK_OPTIONS
        -fprofile-instr-generate
        -fcoverage-mapping)
endif()

function(qtrocket_enable_coverage target_name)
    if(QTROCKET_ENABLE_COVERAGE)
        target_compile_options(${target_name} PRIVATE ${QTROCKET_COVERAGE_COMPILE_OPTIONS})
        get_target_property(qtrocket_target_type ${target_name} TYPE)
        if(NOT qtrocket_target_type STREQUAL "STATIC_LIBRARY")
            target_link_options(${target_name} PRIVATE ${QTROCKET_COVERAGE_LINK_OPTIONS})
        endif()
    endif()
endfunction()

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)


if(WIN32)
  set(CMAKE_PREFIX_PATH $ENV{QTDIR})
#  include_directories("C:\\boost\\boost_1_82_0\\")
#  find_package(Qt6Core REQUIRED)
#  find_package(Qt6Widgets REQUIRED)
endif()
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets LinguistTools PrintSupport)
find_package(Qt6 REQUIRED COMPONENTS Widgets LinguistTools PrintSupport)
#find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools PrintSupport)
#find_package(CURL)
#find_package(fmt)

set(TS_FILES qtrocket_en_US.ts)

include_directories(${PROJECT_SOURCE_DIR})

# This is the main intended application. qtrocket is the full GUI application
# The CLI version is defined separately, and while useful, is mostly for testing
#
# qcustomplot throws some warnings, so silence them so we can turn on -Werror
# for our code
set_source_files_properties(
    gui/qcustomplot.cpp
    PROPERTIES
    COMPILE_OPTIONS "-w"
)
set(PROJECT_SOURCES
        main.cpp
        gui/AboutWindow.cpp
        gui/AboutWindow.h
        gui/AboutWindow.ui
        gui/AnalysisWindow.cpp
        gui/AnalysisWindow.h
        gui/AnalysisWindow.ui
        gui/CannonballTab.cpp
        gui/CannonballTab.h
        gui/CannonballTab.ui
        gui/GuiRunner.cpp
        gui/GuiRunner.h
        gui/MainWindow.cpp
        gui/MainWindow.h
        gui/MainWindow.ui
        gui/RocketModelerView.cpp
        gui/RocketModelerView.h
        gui/RocketTreeView.cpp
        gui/RocketTreeView.h
        gui/SimOptionsTab.cpp
        gui/SimOptionsTab.h
        gui/SimOptionsTab.ui
        gui/ThrustCurveMotorSelector.cpp
        gui/ThrustCurveMotorSelector.h
        gui/ThrustCurveMotorSelector.ui
        gui/qcustomplot.cpp
        gui/qcustomplot.h
        ${TS_FILES}
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(qtrocket
        qtrocket.qrc
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET qtrocket APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(qtrocket SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(qtrocket
            ${PROJECT_SOURCES}
        )
    endif()

endif()
qtrocket_enable_coverage(qtrocket)

add_subdirectory(utils)
add_subdirectory(sim)
add_subdirectory(model)

# Application controller shared by the GUI, the CLI, and the
# integration tests.
add_library(qtrocket_core STATIC
   QtRocket.cpp
   QtRocket.h)
qtrocket_enable_coverage(qtrocket_core)

target_link_libraries(qtrocket_core PUBLIC
   model
   sim
   utils)

# The CLI/REPL driver as a static lib, so both qtrocket-cli and the integration tests can drive it.
add_library(cli STATIC
   cli/Repl.cpp
   cli/Repl.h)
qtrocket_enable_coverage(cli)
target_link_libraries(cli PUBLIC qtrocket_core)

add_subdirectory(tests)

if(QTROCKET_ENABLE_COVERAGE)
    set(QTROCKET_COVERAGE_BINARIES
        "$<TARGET_FILE:sim_tests>"
        "$<TARGET_FILE:model_tests>"
        "$<TARGET_FILE:integration_tests>"
        "$<TARGET_FILE:propagator_tests>"
        "$<TARGET_FILE:cli_tests>")
    list(JOIN QTROCKET_COVERAGE_BINARIES "\\;" QTROCKET_COVERAGE_BINARIES_ARG)

    add_custom_target(coverage
        COMMAND ${CMAKE_COMMAND}
            "-DCOVERAGE_BUILD_DIR=${CMAKE_BINARY_DIR}"
            "-DCOVERAGE_SOURCE_DIR=${CMAKE_SOURCE_DIR}"
            "-DCOVERAGE_BINARIES=${QTROCKET_COVERAGE_BINARIES_ARG}"
            "-DCOVERAGE_TEST_REGEX=^qtrocket_.*tests$"
            "-DCOVERAGE_IGNORE_REGEX=.*/(build|build-coverage|_deps|tests)/.*|.*/gui/qcustomplot.*"
            "-DLLVM_COV_EXECUTABLE=${LLVM_COV_EXECUTABLE}"
            "-DLLVM_PROFDATA_EXECUTABLE=${LLVM_PROFDATA_EXECUTABLE}"
            "-DCTEST_COMMAND=${CMAKE_CTEST_COMMAND}"
            -P "${CMAKE_SOURCE_DIR}/cmake/RunLlvmCoverage.cmake"
        DEPENDS
            sim_tests
            model_tests
            integration_tests
            propagator_tests
            cli_tests
        USES_TERMINAL
        VERBATIM
        COMMENT "Running qtrocket tests and generating LLVM coverage report")
endif()

#target_link_libraries(qtrocket PRIVATE 
#   Qt6::Widgets
#   Qt6::PrintSupport
#   libcurl
#   jsoncpp_static
#   fmt::fmt-header-only
#   eigen)

target_link_libraries(qtrocket PRIVATE
   Qt6::Widgets
   Qt6::PrintSupport
   qtrocket_core)

# Headless CLI / REPL driver for the simulation engine (no Qt). The shared
# application controller comes from the qtrocket_core static lib.
add_executable(qtrocket-cli
    cli/CliMain.cpp
)
qtrocket_enable_coverage(qtrocket-cli)
target_link_libraries(qtrocket-cli PRIVATE
    cli)

set_target_properties(qtrocket PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

install(TARGETS qtrocket
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(qtrocket)
endif()
