cmake_minimum_required(VERSION 3.0.0)
project(
    wxRocket
    VERSION 0.1.0
    DESCRIPTION "Model Rocket Simulation Application"
    LANGUAGES CXX)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include_directories(${PROJECT_SOURCE_DIR}/src)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_STANDARD 20)

    # Need to set specific options for specific compilers here.
    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      # No special flags yet
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # No special flags yet

    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      # Not sure... Anyone that uses MSVC can help here
    endif()

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
add_subdirectory(src/sim/tests)
add_subdirectory(src/utils)
add_subdirectory(src/utils/math)
add_subdirectory(src/model)
