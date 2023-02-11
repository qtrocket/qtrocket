cmake_minimum_required(VERSION 3.0.0)
project(wxRocket VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(wxWidgets REQUIRED gl core base)
include(${wxWidgets_USE_FILE})
add_executable(wxRocket src/main.cpp)

target_link_libraries(wxRocket PRIVATE ${wxWidgets_LIBRARIES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
