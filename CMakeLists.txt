cmake_minimum_required(VERSION 3.16)

project(qtrocket VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

include(FetchContent)

# Google Test framework
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest
    GIT_TAG        v1.13.0)
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
    GIT_TAG        1.9.5)
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
    GIT_TAG        curl-8_0_1)
set(BUILD_CURL_EXE OFF)
set(BUILD_SHARED_LIBS OFF)
set(HTTP_ONLY ON)
set(SSL_ENABLED ON)
if(WIN32)
   set(CURL_USE_SCHANNEL ON)
endif()
FetchContent_MakeAvailable(CURL)

# eigen dependency
FetchContent_Declare(Eigen
   GIT_REPOSITORY https://gitlab.com/libeigen/eigen
   GIT_TAG 3.4.0)
FetchContent_MakeAvailable(Eigen)

# boost dependency

FetchContent_Declare(Boost
   GIT_REPOSITORY https://github.com/boostorg/boost
   GIT_TAG boost-1.84.0)
set(BOOST_INCLUDE_LIBRARIES property_tree)
FetchContent_MakeAvailable(Boost)
# Add qtrocket subdirectories. These are components that will be linked in 


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

set(PROJECT_SOURCES
        main.cpp
        QtRocket.cpp
        QtRocket.h
        gui/AboutWindow.cpp
        gui/AboutWindow.h
        gui/AboutWindow.ui
        gui/AnalysisWindow.cpp
        gui/AnalysisWindow.h
        gui/AnalysisWindow.ui
        gui/MainWindow.cpp
        gui/MainWindow.h
        gui/MainWindow.ui
        gui/RocketTreeView.cpp
        gui/RocketTreeView.h
        gui/SimOptionsWindow.cpp
        gui/SimOptionsWindow.h
        gui/SimOptionsWindow.ui
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

add_subdirectory(utils)
add_subdirectory(sim)
add_subdirectory(model)

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
   utils
   sim
   model)

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
