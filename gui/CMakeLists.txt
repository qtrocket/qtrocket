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
set_source_files_properties(
   qcustomplot.cpp
   PROPERTIES
   COMPILE_OPTIONS "-w"
)

set(SOURCES
   main.cpp
   AboutWindow.cpp
   AboutWindow.h
   AboutWindow.ui
   AnalysisWindow.cpp
   AnalysisWindow.h
   AnalysisWindow.ui
   CannonballTab.cpp
   CannonballTab.h
   CannonballTab.ui
   MainWindow.cpp
   MainWindow.h
   MainWindow.ui
   RocketModelerView.cpp
   RocketModelerView.h
   RocketTreeView.cpp
   RocketTreeView.h
   SimOptionsTab.cpp
   SimOptionsTab.h
   SimOptionsTab.ui
   ThrustCurveMotorSelector.cpp
   ThrustCurveMotorSelector.h
   ThrustCurveMotorSelector.ui
   qcustomplot.cpp
   qcustomplot.h
   ${TS_FILES}
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(qtrocket
        qtrocket.qrc
        MANUAL_FINALIZATION
        ${SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET qtrocket APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(qtrocket SHARED
            ${SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(qtrocket
            ${SOURCES}
        )
    endif()

endif()
qtrocket_enable_coverage(qtrocket)

target_link_libraries(qtrocket PRIVATE
   Qt6::Widgets
   Qt6::PrintSupport
   qtrocket_core
)

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

add_subdirectory(tests)
