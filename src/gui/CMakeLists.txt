find_package(wxWidgets REQUIRED gl core base)
include(${wxWidgets_USE_FILE})
add_executable(wxRocket main.cpp)

target_link_libraries(wxRocket PRIVATE ${wxWidgets_LIBRARIES})
