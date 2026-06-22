# qtrocket-visualizer: a standalone OpenGL viewer for QtRocket design (.qrd) files.
#
# Greenfield utility app. It opens a design through the SHARED loader (model::DesignSerializer,
# linked in from qtrocket_core) and renders the resulting part tree as interactive, user-colorable
# 3D geometry in a core-profile OpenGL viewport. It depends only on Qt + qtrocket_core; it does not
# touch the GUI (gui/) or CLI (cli/) code.

# Qt6::Widgets is already found at the project root; the viewport needs the OpenGL modules too.
find_package(Qt6 REQUIRED COMPONENTS OpenGLWidgets OpenGL)

set(VISUALIZER_SOURCES
    main.cpp
    VisualizerWindow.cpp
    VisualizerWindow.h
    RocketGLWidget.cpp
    RocketGLWidget.h
    RocketMesh.cpp
    RocketMesh.h
    ColorScheme.cpp
    ColorScheme.h)

qt_add_executable(qtrocket-visualizer ${VISUALIZER_SOURCES})

# Project-root include dir (${PROJECT_SOURCE_DIR}) is already on the include path, so the
# "visualizer/..." and "model/..." include prefixes resolve. Eigen comes transitively via
# qtrocket_core -> model -> utils -> eigen.
target_link_libraries(qtrocket-visualizer PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Qt6::OpenGL
    qtrocket_core)
