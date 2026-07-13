// Qt headers
#include <QApplication>
#include <QSurfaceFormat>

// qtrocket headers
#include "visualizer/VisualizerWindow.h"

/// @brief Entry point for the qtrocket-visualizer app: installs a minimal default surface format,
///        opens the main window, and loads an optional .qrd path given on the command line.
///
/// The default format deliberately requests no GL version, core profile, or MSAA. Forcing a 3.3
/// core profile as the app-wide default poisons every context Qt creates, including the GLES2
/// widget-compositor context the RHI uses; on EGL (Wayland + NVIDIA) the driver then has no
/// matching window EGLConfig and context creation fails with EGL_BAD_MATCH (the 3D view goes
/// black), while GLX honors the same request -- hence it runs on one machine but not another.
/// Left unconstrained, Qt picks whatever the platform can create and RocketGLWidget adapts its
/// shaders to it; the viewport still anti-aliases via GL_MULTISAMPLE per frame.
int main(int argc, char* argv[])
{
    // Disable the RHI widget backing store; on EGL/NVIDIA it otherwise tries (and fails) to create
    // a GLES2 RHI context for QBackingStoreRhiSupport before our viewport comes up.
    qputenv("QT_WIDGETS_RHI", "0");

    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(24);
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    QApplication::setApplicationName("QtRocket Visualizer");

    viz::VisualizerWindow w;
    w.resize(1200, 800);
    w.show();

    if (argc > 1)
    {
        w.openFile(QString::fromLocal8Bit(argv[1]));
    }

    return app.exec();
}
