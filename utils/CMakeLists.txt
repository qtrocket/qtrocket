add_library(utils
    Bin.cpp
    Bin.h
    CurlConnection.cpp
    CurlConnection.h
    Logger.cpp
    Logger.h
    MotorModelDatabase.cpp
    MotorModelDatabase.h
    RSEDatabaseLoader.cpp
    RSEDatabaseLoader.h
    ThrustCurveAPI.cpp
    ThrustCurveAPI.h
    math/Constants.h
    math/MathTypes.h
    math/UtilityMathFunctions.h)


#target_include_directories(utils PRIVATE
#    ${Boost_INCLUDE_DIRS})


# 'model' creates a declared cycle (model links utils for Logger/math; utils
# links model for MotorModel/ThrustCurve in the database/API code). CMake
# breaks static-library cycles by repeating the archives on the link line.
target_link_libraries(utils PUBLIC
   libcurl
   Boost::property_tree
   jsoncpp_static
   eigen
   model)

add_subdirectory(tests)
