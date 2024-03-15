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
        ThreadPool.cpp
        ThreadPool.h
        ThrustCurveAPI.cpp
        ThrustCurveAPI.h
        TSQueue.h
        math/Constants.h
        math/MathTypes.h
        math/UtilityMathFunctions.h)


#target_include_directories(utils PRIVATE
#    ${Boost_INCLUDE_DIRS})


target_link_libraries(utils PUBLIC
   libcurl
   Boost::property_tree
   jsoncpp_static
   eigen)
