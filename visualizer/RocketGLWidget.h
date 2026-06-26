#ifndef VISUALIZER_ROCKETGLWIDGET_H
#define VISUALIZER_ROCKETGLWIDGET_H

// Qt headers
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QVector3D>
#include <QPoint>

// C++ headers
#include <memory>
#include <vector>

// qtrocket headers
#include "visualizer/ColorScheme.h"
#include "visualizer/RocketMesh.h"

class QOpenGLShaderProgram;
class QMouseEvent;
class QWheelEvent;

namespace viz
{

/// @brief Interactive OpenGL viewport that renders a rocket as shaded 3D geometry.
///
/// Uses the version-agnostic QOpenGLFunctions (common GL/GLES2 subset) so it runs on whatever
/// context the platform yields (see main.cpp); initializeGL() picks GLSL 1.20 or GLSL ES source.
/// One lit shader for components (per-component color, one headlight + ambient), one unlit shader
/// for the grid and axes. Orbit camera: left-drag rotates, right/middle pans, wheel zooms; model
/// shown nose-up. Wireframe is explicit GL_LINES, not glPolygonMode (unavailable on GLES2). GPU
/// resources live from initializeGL()/setRenderItems() to the destructor/cleanup(); the widget
/// owns all it allocates.
class RocketGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
   Q_OBJECT

public:
   explicit RocketGLWidget(QWidget* parent = nullptr);
   ~RocketGLWidget() override;

   /// @brief Replace the rendered geometry. Uploads each item to its own VAO/VBO/IBO, records the
   ///        per-item type for coloring, recomputes scene bounds, and frames the camera to fit.
   ///        Safe to call before or after the GL context exists (defers GPU upload until it does).
   void setRenderItems(std::vector<RenderItem> items);

   /// @brief Apply a color scheme: recolors every component by its part type and repaints. Also
   ///        updates the viewport background.
   void setColorScheme(const ColorScheme& scheme);

   /// @brief The currently active color scheme (so the window can seed its per-type pickers).
   const ColorScheme& colorScheme() const { return scheme; }

public slots:
   void resetCamera();              ///< Re-frame the camera to fit the current scene bounds.
   void setWireframe(bool on);      ///< Toggle GL_LINE polygon mode for the rocket geometry.
   void setShowGrid(bool on);       ///< Toggle the reference ground grid.
   void setShowAxes(bool on);       ///< Toggle the origin XYZ axis triad.

signals:
   /// @brief Emitted once after initializeGL() with the GL renderer + version strings (for the
   ///        status bar / diagnostics).
   void rendererInfo(const QString& renderer, const QString& version);

protected:
   void initializeGL() override;
   void resizeGL(int w, int h) override;
   void paintGL() override;

   void mousePressEvent(QMouseEvent* e) override;
   void mouseMoveEvent(QMouseEvent* e) override;
   void wheelEvent(QWheelEvent* e) override;

private:
   /// @brief One uploaded component: its GPU buffers, index count, part-type tag, and resolved color.
   ///        Carries a second VAO/VBO of expanded triangle edges (GL_LINES) for wireframe mode,
   ///        since glPolygonMode is unavailable on GLES2 / the generic QOpenGLFunctions.
   struct GpuMesh
   {
      QOpenGLVertexArrayObject vao;
      QOpenGLBuffer            vbo{QOpenGLBuffer::VertexBuffer};
      QOpenGLBuffer            ibo{QOpenGLBuffer::IndexBuffer};
      int                      indexCount{0};
      QOpenGLVertexArrayObject wireVao;
      QOpenGLBuffer            wireVbo{QOpenGLBuffer::VertexBuffer};
      int                      wireVertexCount{0};    ///< expanded edge vertices (3 edges/triangle)
      QString                  typeName;          ///< ColorScheme lookup key
      QVector3D                color{0.7F, 0.7F, 0.7F}; ///< resolved RGB in [0,1]
      bool                     overlapOffender{false}; ///< diagnostics flagged this part; render in the error color
   };

   /// @brief Upload @ref items into @ref meshes (requires a current GL context). Clears any prior
   ///        GPU meshes first. Called from setRenderItems() once the context exists.
   void uploadMeshes();
   /// @brief Build/refresh the grid + axes line buffers (requires a current GL context).
   void buildOverlays();
   /// @brief Resolve each GpuMesh's color from @ref scheme by its part type.
   void applySchemeColors();
   /// @brief Release all GPU resources (idempotent; makes the context current as needed).
   void cleanup();
   /// @brief Current view-projection matrix from the orbit camera + viewport aspect.
   QMatrix4x4 viewProjection() const;

   // --- shaders ---
   std::unique_ptr<QOpenGLShaderProgram> litProgram;  ///< rocket geometry (Lambert + ambient)
   std::unique_ptr<QOpenGLShaderProgram> lineProgram; ///< unlit grid + axes

   // --- geometry ---
   std::vector<RenderItem>               items;   ///< CPU copy (for deferred upload / re-frame)
   std::vector<std::unique_ptr<GpuMesh>> meshes;  ///< uploaded components
   Bounds                                bounds;  ///< scene bounds (model space)

   // --- overlays ---
   QOpenGLVertexArrayObject gridVao;
   QOpenGLBuffer            gridVbo{QOpenGLBuffer::VertexBuffer};
   int                      gridVertexCount{0};
   QOpenGLVertexArrayObject axesVao;
   QOpenGLBuffer            axesVbo{QOpenGLBuffer::VertexBuffer};
   int                      axesVertexCount{0};

   // --- appearance ---
   ColorScheme scheme;

   // --- orbit camera ---
   float     camYaw{35.0F};      ///< degrees, around the vertical (up) axis
   float     camPitch{20.0F};    ///< degrees, elevation
   float     camDistance{1.0F};  ///< distance from target
   QVector3D camTarget{0.0F, 0.0F, 0.0F};

   // --- interaction / toggles ---
   QPoint lastMousePos;
   bool   wireframe{false};
   bool   showGrid{true};
   bool   showAxes{true};
   bool   glReady{false};        ///< true once initializeGL() has run
};

} // namespace viz

#endif // VISUALIZER_ROCKETGLWIDGET_H
