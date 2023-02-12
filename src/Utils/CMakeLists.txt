find_package(CURL)

add_library(utils
   CurlConnection.cpp)

target_link_libraries(utils curl jsoncpp ssl crypto) 
