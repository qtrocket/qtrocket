add_library(utils
        BinMap.cpp
        BinMap.h
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

target_link_libraries(utils PUBLIC
   libcurl
   jsoncpp_static
   fmt::fmt-header-only
   eigen)
