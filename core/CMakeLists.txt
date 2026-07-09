# Application controller shared by the GUI, the CLI, and the
# integration tests.
add_library(qtrocket_core STATIC
   QtRocket.cpp
   QtRocket.h)
qtrocket_enable_coverage(qtrocket_core)

target_link_libraries(qtrocket_core PUBLIC
   model
   sim
   utils)