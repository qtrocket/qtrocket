QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    QtRocket.cpp \
    gui/AboutWindow.cpp \
    gui/AnalysisWindow.cpp \
    gui/SimulationOptions.cpp \
    gui/ThrustCurveMotorSelector.cpp \
    gui/qcustomplot.cpp \
    main.cpp \
    gui/RocketTreeView.cpp \
    gui/MainWindow.cpp \
    model/MotorCase.cpp \
    model/MotorModel.cpp \
    model/MotorModelDatabase.cpp \
    model/Rocket.cpp \
    model/ThrustCurve.cpp \
    sim/ConstantGravityModel.cpp \
    sim/GravityModel.cpp \
    sim/Propagator.cpp \
    sim/SphericalGeoidModel.cpp \
    sim/SphericalGravityModel.cpp \
    sim/StateData.cpp \
    sim/USStandardAtmosphere.cpp \
    sim/WindModel.cpp \
    utils/BinMap.cpp \
    utils/CurlConnection.cpp \
    utils/Logger.cpp \
    utils/RSEDatabaseLoader.cpp \
    utils/ThreadPool.cpp \
    utils/ThrustCurveAPI.cpp \
    utils/math/Quaternion.cpp \
    utils/math/UtilityMathFunctions.cpp \
    utils/math/Vector3.cpp

HEADERS += \
    QtRocket.h \
    gui/AboutWindow.h \
    gui/AnalysisWindow.h \
    gui/RocketTreeView.h \
    gui/MainWindow.h \
    gui/SimulationOptions.h \
    gui/ThrustCurveMotorSelector.h \
    gui/qcustomplot.h \
    model/MotorCase.h \
    model/MotorModel.h \
    model/MotorModelDatabase.h \
    model/Rocket.h \
    model/ThrustCurve.h \
    sim/AtmosphericModel.h \
    sim/ConstantAtmosphere.h \
    sim/ConstantGravityModel.h \
    sim/DESolver.h \
    sim/GeoidModel.h \
    sim/GravityModel.h \
    sim/Propagator.h \
    sim/RK4Solver.h \
    sim/SphericalGeoidModel.h \
    sim/SphericalGravityModel.h \
    sim/StateData.h \
    sim/USStandardAtmosphere.h \
    sim/WindModel.h \
    utils/BinMap.h \
    utils/CurlConnection.h \
    utils/Logger.h \
    utils/RSEDatabaseLoader.h \
    utils/TSQueue.h \
    utils/ThreadPool.h \
    utils/ThrustCurveAPI.h \
    utils/Triplet.h \
    utils/math/Constants.h \
    utils/math/Quaternion.h \
    utils/math/UtilityMathFunctions.h \
    utils/math/Vector3.h

FORMS += \
    gui/AboutWindow.ui \
    gui/AnalysisWindow.ui \
    gui/MainWindow.ui \
    gui/SimulationOptions.ui \
    gui/ThrustCurveMotorSelector.ui

TRANSLATIONS += \
    qtrocket_en_US.ts
CONFIG += lrelease
CONFIG += embed_translations

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += libcurl
unix: PKGCONFIG += fmt
unix: PKGCONFIG += jsoncpp

RESOURCES += \
   qtrocket.qrc
