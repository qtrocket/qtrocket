cmake_minimum_required(VERSION 3.16)

project(qtrocket VERSION 0.1 LANGUAGES CXX)


include(FetchContent)

# fmtlib dependency
FetchContent_Declare(fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt
    GIT_TAG        9.1.0)
FetchContent_MakeAvailable(fmt)

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



set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
  set(CMAKE_PREFIX_PATH $ENV{QTDIR})
  include_directories("C:\\boost\\boost_1_82_0\\")
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
        model/MotorCase.cpp
        model/MotorCase.h
        model/MotorModel.cpp
        model/MotorModel.h
        model/MotorModelDatabase.cpp
        model/MotorModelDatabase.h
        model/Rocket.cpp
        model/Rocket.h
        model/ThrustCurve.cpp
        model/ThrustCurve.h
        sim/AtmosphericModel.h
        sim/ConstantAtmosphere.h
        sim/ConstantGravityModel.cpp
        sim/ConstantGravityModel.h
        sim/DESolver.h
        sim/Environment.h
        sim/GeoidModel.h
        sim/GravityModel.cpp
        sim/GravityModel.h
        sim/Propagator.cpp
        sim/Propagator.h
        sim/RK4Solver.h
        sim/SphericalGeoidModel.cpp
        sim/SphericalGeoidModel.h
        sim/SphericalGravityModel.cpp
        sim/SphericalGravityModel.h
        sim/StateData.cpp
        sim/StateData.h
        sim/USStandardAtmosphere.cpp
        sim/USStandardAtmosphere.h
        sim/WindModel.cpp
        sim/WindModel.h
        utils/BinMap.cpp
        utils/BinMap.h
        utils/CurlConnection.cpp
        utils/CurlConnection.h
        utils/Logger.cpp
        utils/Logger.h
        utils/MotorModelDatabase.cpp
        utils/MotorModelDatabase.h
        utils/RSEDatabaseLoader.cpp
        utils/RSEDatabaseLoader.h
        utils/ThreadPool.cpp
        utils/ThreadPool.h
        utils/ThrustCurveAPI.cpp
        utils/ThrustCurveAPI.h
        utils/Triplet.h
        utils/TSQueue.h
        utils/math/Constants.h
        utils/math/Quaternion.cpp
        utils/math/Quaternion.h
        utils/math/UtilityMathFunctions.cpp
        utils/math/UtilityMathFunctions.h
        utils/math/Vector3.cpp
        utils/math/Vector3.h
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

    #qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

#target_link_libraries(qtrocket PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${Qt_VERSION_MAJOR}::PrintSupport libcurl jsoncpp_static fmt::fmt-header-only)
target_link_libraries(qtrocket PRIVATE Qt6::Widgets Qt6::PrintSupport libcurl jsoncpp_static fmt::fmt-header-only)

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
