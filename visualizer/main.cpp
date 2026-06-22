// Qt headers
#include <QApplication>
#include <QSurfaceFormat>

// qtrocket headers
#include "visualizer/VisualizerWindow.h"

/**
 * @brief Entry point for the qtrocket-visualizer app.
 *
 * Installs an OpenGL 3.3 core-profile default surface format (with a depth buffer) before
 * constructing the QApplication, then opens the main visualizer window. An optional command line
 * argument is treated as a .qrd design file to load on startup.
 *
 * NOTE: the default format is kept deliberately minimal (no multisampling request). Asking for MSAA
 * here forces EVERY context Qt creates -- including the GLES2 widget-compositor/backing-store
 * context -- to find a window EGLConfig with that sample count; on EGL paths (e.g. Wayland/Xwayland
 * with the NVIDIA driver) that exposes no multisampled window config, context creation then fails
 * with EGL_BAD_MATCH (0x3009) and the 3D view goes black. The viewport instead anti-aliases per
 * frame via GL_MULTISAMPLE on whatever the context provides (see RocketGLWidget::initializeGL).
 */
int main(int argc, char* argv[])
{
   QSurfaceFormat fmt;
   fmt.setVersion(3, 3);
   fmt.setProfile(QSurfaceFormat::CoreProfile);
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
