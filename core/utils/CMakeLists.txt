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

# headers are included as "utils/...", rooted at core/
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Bottom layer: utils must not depend on model/ or sim/.
target_link_libraries(utils PUBLIC
   libcurl
   eigen)
