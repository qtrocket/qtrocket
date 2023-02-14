cmake_minimum_required(VERSION 3.0.0)
project(
    wxRocket
    VERSION 0.1.0
    DESCRIPTION "Model Rocket Simulation Application"
    LANGUAGES CXX)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_STANDARD 17)

    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

    include(CTest)

    #find_package(Doxygen)
    #if(Doxygen_FOUND)
    #    add_subdirectory(docs)
    #else()
    #    message(STATUS "Doxygen not found, not building docs")
    #endif()
endif()

enable_testing()

add_subdirectory(src/gui)
add_subdirectory(src/sim)
add_subdirectory(src/Utils)
add_subdirectory(src/Utils/math)
add_subdirectory(src/model)

#find_package(wxWidgets REQUIRED gl core base)
#include(${wxWidgets_USE_FILE})
#add_executable(wxRocket src/main.cpp)

#target_link_libraries(wxRocket PRIVATE ${wxWidgets_LIBRARIES})

#set(CPACK_PROJECT_NAME ${PROJECT_NAME})
#set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
#include(CPack)
