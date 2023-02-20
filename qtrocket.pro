QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    QtRocket.cpp \
    model/Motor.cpp \
    model/MotorCase.cpp \
    model/Thrustcurve.cpp \
    qcustomplot.cpp \
    sim/AtmosphericModel.cpp \
    sim/GravityModel.cpp \
    sim/SphericalGeoidModel.cpp \
    sim/SphericalGravityModel.cpp \
    sim/USStandardAtmosphere.cpp \
    sim/WindModel.cpp \
    utils/BinMap.cpp \
    utils/CurlConnection.cpp \
    utils/ThrustCurveAPI.cpp

HEADERS += \
    QtRocket.h \
    model/Motor.h \
    model/MotorCase.h \
    model/Thrustcurve.h \
    qcustomplot.h \
    sim/AtmosphericModel.h \
    sim/GeoidModel.h \
    sim/GravityModel.h \
    sim/SphericalGeoidModel.h \
    sim/SphericalGravityModel.h \
    sim/USStandardAtmosphere.h \
    sim/WindModel.h \
    utils/BinMap.h \
    utils/CurlConnection.h \
    utils/ThrustCurveAPI.h \
    utils/math/Constants.h

FORMS += \
    QtRocket.ui

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
