add_library(utils
    Bin.cpp
    Bin.h
    CurlConnection.cpp
    CurlConnection.h
    Logger.cpp
    Logger.h
    math/Constants.h
    math/MathTypes.h
    math/UtilityMathFunctions.h)
qtrocket_enable_coverage(utils)

# Bottom layer: utils must not depend on model/ or sim/.
target_link_libraries(utils PUBLIC
   libcurl
   eigen)
