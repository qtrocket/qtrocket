# The CLI/REPL driver as a static lib, so both qtrocket-cli and the integration tests can drive it.
add_library(cli STATIC
   Repl.cpp
   Repl.h)
qtrocket_enable_coverage(cli)
target_link_libraries(cli PUBLIC qtrocket_core)

# Headless CLI / REPL driver for the simulation engine (no Qt). The shared
# application controller comes from the qtrocket_core static lib.
add_executable(qtrocket-cli
    CliMain.cpp)
qtrocket_enable_coverage(qtrocket-cli)
target_compile_definitions(qtrocket-cli PRIVATE
    QTROCKET_VERSION="${PROJECT_VERSION}")
target_link_libraries(qtrocket-cli PRIVATE
    cli)
