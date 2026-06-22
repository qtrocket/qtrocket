// Qt headers
#include <QApplication>
#include <QSurfaceFormat>

// qtrocket headers
#include "visualizer/VisualizerWindow.h"

/**
 * @brief Entry point for the qtrocket-visualizer app.
 *
 * Installs a deliberately minimal default surface format (just a depth buffer) before constructing
 * the QApplication, then opens the main visualizer window. An optional command line argument is
 * treated as a .qrd design file to load on startup.
 *
 * NOTE: the default format intentionally does NOT request a specific GL version, a core profile, or
 * multisampling. Forcing a 3.3 core profile (or MSAA) as the application-wide default poisons EVERY
 * context Qt creates -- including the GLES2 widget-compositor / backing-store context that the Qt
 * RHI uses. On EGL paths (e.g. Wayland with the NVIDIA driver) the driver then exposes no matching
 * window EGLConfig, so context creation fails with EGL_BAD_MATCH (0x3009) and the 3D view goes
 * black -- while GLX/desktop drivers happily honor the same request, which is why it can run on one
 * machine but not another. Leaving the format unconstrained lets Qt pick whatever the platform can
 * actually create (a desktop compatibility context or a GLES2 context); RocketGLWidget adapts its
 * shaders to whichever it gets. We also disable the RHI widget backing store (QT_WIDGETS_RHI=0) so
 * the widget compositor never tries to spin up its own GLES2 RHI context. The viewport still
 * anti-aliases per frame via GL_MULTISAMPLE on whatever the context provides (see
 * RocketGLWidget::initializeGL).
 */
int main(int argc, char* argv[])
{
   // Disable the RHI-based widget backing store; on EGL/NVIDIA it otherwise tries (and fails) to
   // create a GLES2 RHI context for QBackingStoreRhiSupport before our viewport even comes up.
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
