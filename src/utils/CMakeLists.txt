find_package(CURL)
find_package(fmt)

add_library(utils
   BinMap.cpp
   CurlConnection.cpp)

# Also including the header-only fmt library
target_link_libraries(utils PRIVATE curl jsoncpp ssl crypto fmt::fmt-header-only) 
