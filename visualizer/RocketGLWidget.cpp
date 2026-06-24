// Corresponding header
#include "visualizer/RocketGLWidget.h"

// Qt headers
#include <QOpenGLShaderProgram>
#include <QOpenGLContext>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QColor>
#include <QtMath>

// C++ headers
#include <algorithm>
#include <cmath>
#include <vector>

namespace viz
{

namespace
{

// Shaders are written for the lowest-common-denominator dialects so they run on whatever context
// main.cpp's unconstrained surface format yields: GLSL 1.20 on a desktop compatibility context, or
// GLSL ES 1.00 on a GLES2 context. Both use attribute/varying and gl_FragColor (no in/out, no
// layout qualifiers); attribute locations are bound explicitly in buildProgram(). initializeGL()
// picks the desktop or ES pair via QOpenGLContext::isOpenGLES().

/// @brief Vertex shader for the lit rocket geometry (desktop GLSL 1.20): transforms position by the
///        MVP and passes the model-space normal to the fragment stage.
constexpr const char* kLitVertDesktop = R"(#version 120
attribute vec3 aPos;
attribute vec3 aNormal;

uniform mat4 uMvp;
uniform mat4 uModel;

varying vec3 vNormal;

void main()
{
   gl_Position = uMvp * vec4(aPos, 1.0);
   vNormal     = mat3(uModel) * aNormal;
}
)";

/// @brief Fragment shader (desktop GLSL 1.20): single directional headlight, Lambert diffuse +
///        healthy ambient + mild specular. Two-sided shading so thin fins and tube interiors read.
constexpr const char* kLitFragDesktop = R"(#version 120
varying vec3 vNormal;

uniform vec3 uColor;
uniform vec3 uLightDir;

void main()
{
   vec3 n = normalize(vNormal);
   vec3 l = normalize(-uLightDir);

   // Two-sided: flip the normal toward the light so back faces are lit too.
   if (dot(n, l) < 0.0)
      n = -n;

   float ambient = 0.35;
   float diffuse = max(dot(n, l), 0.0);

   // Mild specular using the same direction as a cheap camera-aligned headlight.
   vec3  viewDir = vec3(0.0, 0.0, 1.0);
   vec3  halfVec = normalize(l + viewDir);
   float spec    = pow(max(dot(n, halfVec), 0.0), 24.0) * 0.20;

   vec3 color = uColor * (ambient + 0.65 * diffuse) + vec3(spec);
   gl_FragColor = vec4(color, 1.0);
}
)";

/// @brief Vertex shader for the unlit overlays (grid + axes), desktop GLSL 1.20.
constexpr const char* kLineVertDesktop = R"(#version 120
attribute vec3 aPos;

uniform mat4 uMvp;

void main()
{
   gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

/// @brief Fragment shader for the unlit overlays: a flat color (desktop GLSL 1.20).
constexpr const char* kLineFragDesktop = R"(#version 120
uniform vec3 uColor;

void main()
{
   gl_FragColor = vec4(uColor, 1.0);
}
)";

/// @brief GLSL ES 1.00 twin of @ref kLitVertDesktop (no #version => defaults to ES 1.00).
constexpr const char* kLitVertEs = R"(attribute vec3 aPos;
attribute vec3 aNormal;

uniform mat4 uMvp;
uniform mat4 uModel;

varying vec3 vNormal;

void main()
{
   gl_Position = uMvp * vec4(aPos, 1.0);
   vNormal     = mat3(uModel) * aNormal;
}
)";

/// @brief GLSL ES 1.00 twin of @ref kLitFragDesktop (adds the required float precision qualifier).
constexpr const char* kLitFragEs = R"(precision mediump float;
varying vec3 vNormal;

uniform vec3 uColor;
uniform vec3 uLightDir;

void main()
{
   vec3 n = normalize(vNormal);
   vec3 l = normalize(-uLightDir);

   if (dot(n, l) < 0.0)
      n = -n;

   float ambient = 0.35;
   float diffuse = max(dot(n, l), 0.0);

   vec3  viewDir = vec3(0.0, 0.0, 1.0);
   vec3  halfVec = normalize(l + viewDir);
   float spec    = pow(max(dot(n, halfVec), 0.0), 24.0) * 0.20;

   vec3 color = uColor * (ambient + 0.65 * diffuse) + vec3(spec);
   gl_FragColor = vec4(color, 1.0);
}
)";

/// @brief GLSL ES 1.00 twin of @ref kLineVertDesktop.
constexpr const char* kLineVertEs = R"(attribute vec3 aPos;

uniform mat4 uMvp;

void main()
{
   gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

/// @brief GLSL ES 1.00 twin of @ref kLineFragDesktop.
constexpr const char* kLineFragEs = R"(precision mediump float;
uniform vec3 uColor;

void main()
{
   gl_FragColor = vec4(uColor, 1.0);
}
)";

/// @brief Convert a QColor to a linear-ish RGB QVector3D in [0,1].
QVector3D qcolorToVec3(const QColor& c)
{
   return QVector3D(static_cast<float>(c.redF()),
                    static_cast<float>(c.greenF()),
                    static_cast<float>(c.blueF()));
}

/// @brief The "stand-up" model transform applied to the rocket only: maps the body-frame long axis
///        (+z, nose) to world up (+Y). The ground grid and axes stay in world space (not rotated),
///        so any world-space point must be mapped through this to follow the rocket.
QMatrix4x4 standUpMatrix()
{
   QMatrix4x4 m;
   m.rotate(-90.0F, 1.0F, 0.0F, 0.0F);
   return m;
}

/// @brief Compile + link a two-stage shader program; qWarning on failure. Returns nullptr on error.
///        Attribute locations are bound by name (GLSL 1.20 / ES 1.00 have no layout qualifiers) to
///        the fixed slots the VBO setup uses: aPos -> 0, aNormal -> 1. Binding a name a given shader
///        does not declare (e.g. aNormal in the line shader) is harmless.
std::unique_ptr<QOpenGLShaderProgram> buildProgram(const char* vertSrc, const char* fragSrc,
                                                   const char* label)
{
   auto prog = std::make_unique<QOpenGLShaderProgram>();
   if (!prog->addShaderFromSourceCode(QOpenGLShader::Vertex, vertSrc))
   {
      qWarning("RocketGLWidget: %s vertex shader failed: %s", label,
               prog->log().toUtf8().constData());
      return nullptr;
   }
   if (!prog->addShaderFromSourceCode(QOpenGLShader::Fragment, fragSrc))
   {
      qWarning("RocketGLWidget: %s fragment shader failed: %s", label,
               prog->log().toUtf8().constData());
      return nullptr;
   }
   prog->bindAttributeLocation("aPos", 0);
   prog->bindAttributeLocation("aNormal", 1);
   if (!prog->link())
   {
      qWarning("RocketGLWidget: %s shader link failed: %s", label,
               prog->log().toUtf8().constData());
      return nullptr;
   }
   return prog;
}

} // namespace

RocketGLWidget::RocketGLWidget(QWidget* parent)
   : QOpenGLWidget(parent),
     scheme(defaultScheme())
{
}

RocketGLWidget::~RocketGLWidget()
{
   // GL teardown requires a current context; guard against having never initialized.
   makeCurrent();
   cleanup();
   doneCurrent();
}

void RocketGLWidget::setRenderItems(std::vector<RenderItem> newItems)
{
   items  = std::move(newItems);
   bounds = computeBounds(items);

   if (glReady)
   {
      makeCurrent();
      uploadMeshes();
      buildOverlays(); // re-size the grid/axes to the freshly computed bounds
      doneCurrent();
      resetCamera();
      update();
   }
   // Otherwise the upload is deferred until the end of initializeGL().
}

void RocketGLWidget::setColorScheme(const ColorScheme& newScheme)
{
   scheme = newScheme;
   if (glReady)
   {
      makeCurrent();
      applySchemeColors();
      doneCurrent();
   }
   else
   {
      applySchemeColors();
   }
   update();
}

void RocketGLWidget::resetCamera()
{
   // The rocket is drawn through the stand-up transform, so the orbit target is the rocket's
   // WORLD-space center (model-space center mapped through that same rotation), not the raw
   // model-space center -- otherwise the camera aims off to one side.
   camTarget = standUpMatrix().map(bounds.center());

   // Frame the whole scene: pull back far enough that the bounding sphere fits inside the view
   // frustum, using whichever of the vertical/horizontal half-FOV is tighter (so tall, narrow
   // rockets fit in a wide window and vice-versa), plus a small margin. Falls back to a square
   // aspect before the first resize.
   const float halfFovY = qDegreesToRadians(45.0F * 0.5F);
   const float aspect   = (height() > 0) ? static_cast<float>(width()) / static_cast<float>(height())
                                         : 1.0F;
   const float halfFovX = std::atan(std::tan(halfFovY) * aspect);
   const float halfFov  = std::min(halfFovY, halfFovX);
   camDistance = std::max(bounds.radius() / std::sin(halfFov) * 1.15F, 0.05F);

   camYaw   = 35.0F;
   camPitch = 20.0F;
   update();
}

void RocketGLWidget::setWireframe(bool on)
{
   wireframe = on;
   update();
}

void RocketGLWidget::setShowGrid(bool on)
{
   showGrid = on;
   update();
}

void RocketGLWidget::setShowAxes(bool on)
{
   showAxes = on;
   update();
}

void RocketGLWidget::initializeGL()
{
   initializeOpenGLFunctions();

   glEnable(GL_DEPTH_TEST);
   // Culling intentionally disabled: thin double-sided fins and visible tube interiors need both
   // faces drawn. The lit shader does two-sided shading to compensate.
   glDisable(GL_CULL_FACE);
   glEnable(GL_MULTISAMPLE);

   const QColor& bg = scheme.background;
   glClearColor(static_cast<float>(bg.redF()), static_cast<float>(bg.greenF()),
                static_cast<float>(bg.blueF()), 1.0F);

   // The unconstrained surface format (see main.cpp) may yield either a desktop compatibility
   // context or a GLES2 context; compile the matching shader dialect for whichever we got.
   const bool gles = context() != nullptr && context()->isOpenGLES();
   litProgram  = buildProgram(gles ? kLitVertEs : kLitVertDesktop,
                              gles ? kLitFragEs : kLitFragDesktop, "lit");
   lineProgram = buildProgram(gles ? kLineVertEs : kLineVertDesktop,
                              gles ? kLineFragEs : kLineFragDesktop, "line");

   glReady = true;

   // Any geometry queued before the context existed gets uploaded now.
   if (!items.empty())
   {
      uploadMeshes();
      resetCamera();
   }
   buildOverlays();

   const GLubyte* renderer = glGetString(GL_RENDERER);
   const GLubyte* version  = glGetString(GL_VERSION);
   emit rendererInfo(QString::fromUtf8(renderer ? reinterpret_cast<const char*>(renderer) : ""),
                     QString::fromUtf8(version ? reinterpret_cast<const char*>(version) : ""));
}

void RocketGLWidget::resizeGL(int w, int h)
{
   glViewport(0, 0, w, h);
}

void RocketGLWidget::paintGL()
{
   const QColor& bg = scheme.background;
   glClearColor(static_cast<float>(bg.redF()), static_cast<float>(bg.greenF()),
                static_cast<float>(bg.blueF()), 1.0F);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   // Stand the rocket up: body +z -> world +Y. The grid/axes live in world space and are NOT
   // rotated, so the ground stays flat and the Y axis truly points up.
   const QMatrix4x4 model = standUpMatrix();
   const QMatrix4x4 vp    = viewProjection();
   const QMatrix4x4 mvp   = vp * model;

   // --- overlays (unlit, world space) ---
   if (lineProgram && (showGrid || showAxes))
   {
      lineProgram->bind();
      lineProgram->setUniformValue("uMvp", vp);

      if (showGrid && gridVertexCount > 0)
      {
         lineProgram->setUniformValue("uColor", QVector3D(0.40F, 0.43F, 0.48F));
         gridVao.bind();
         glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(gridVertexCount));
         gridVao.release();
      }

      if (showAxes && axesVertexCount >= 6)
      {
         // Draw each axis segment with its own color so the triad reads X/Y/Z distinctly.
         axesVao.bind();
         lineProgram->setUniformValue("uColor", QVector3D(0.85F, 0.25F, 0.25F)); // X = red
         glDrawArrays(GL_LINES, 0, 2);
         lineProgram->setUniformValue("uColor", QVector3D(0.30F, 0.80F, 0.30F)); // Y = green
         glDrawArrays(GL_LINES, 2, 2);
         lineProgram->setUniformValue("uColor", QVector3D(0.35F, 0.55F, 0.95F)); // Z = blue
         glDrawArrays(GL_LINES, 4, 2);
         axesVao.release();
      }

      lineProgram->release();
   }

   // --- rocket geometry (lit) ---
   if (litProgram && !meshes.empty())
   {
      litProgram->bind();
      litProgram->setUniformValue("uMvp", mvp);
      litProgram->setUniformValue("uModel", model);
      litProgram->setUniformValue("uLightDir", QVector3D(-0.4F, -0.6F, -0.7F));

      for (const auto& meshPtr : meshes)
      {
         GpuMesh& gm = *meshPtr;
         litProgram->setUniformValue("uColor", gm.color);

         // Wireframe draws the pre-expanded edge buffer as GL_LINES; solid draws indexed triangles.
         if (wireframe)
         {
            if (gm.wireVertexCount == 0)
               continue;
            gm.wireVao.bind();
            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(gm.wireVertexCount));
            gm.wireVao.release();
         }
         else
         {
            if (gm.indexCount == 0)
               continue;
            gm.vao.bind();
            glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(gm.indexCount), GL_UNSIGNED_INT,
                           nullptr);
            gm.vao.release();
         }
      }

      litProgram->release();
   }
}

void RocketGLWidget::mousePressEvent(QMouseEvent* e)
{
   lastMousePos = e->pos();
}

void RocketGLWidget::mouseMoveEvent(QMouseEvent* e)
{
   const QPoint delta = e->pos() - lastMousePos;
   lastMousePos       = e->pos();

   const float dx = static_cast<float>(delta.x());
   const float dy = static_cast<float>(delta.y());

   if (e->buttons() & Qt::LeftButton)
   {
      // Orbit: yaw/pitch.
      camYaw += dx * 0.4F;
      camPitch += dy * 0.4F;
      camPitch = std::clamp(camPitch, -89.0F, 89.0F);
      update();
   }
   else if (e->buttons() & (Qt::RightButton | Qt::MiddleButton))
   {
      // Pan target in the camera's right/up plane, scaled by distance.
      const float yawR   = qDegreesToRadians(camYaw);
      const float pitchR = qDegreesToRadians(camPitch);

      const QVector3D dir(std::cos(pitchR) * std::sin(yawR),
                          std::sin(pitchR),
                          std::cos(pitchR) * std::cos(yawR));
      const QVector3D worldUp(0.0F, 1.0F, 0.0F);
      QVector3D       right = QVector3D::crossProduct(dir, worldUp);
      if (right.lengthSquared() < 1e-8F)
         right = QVector3D(1.0F, 0.0F, 0.0F);
      right.normalize();
      const QVector3D up = QVector3D::crossProduct(right, dir).normalized();

      const float panScale = camDistance * 0.0025F;
      camTarget += (-right * dx + up * dy) * panScale;
      update();
   }
}

void RocketGLWidget::wheelEvent(QWheelEvent* e)
{
   const float steps = static_cast<float>(e->angleDelta().y()) / 120.0F;
   const float factor = std::pow(0.9F, steps);
   camDistance        = std::clamp(camDistance * factor,
                            std::max(bounds.radius() * 0.05F, 1e-3F),
                            std::max(bounds.radius() * 50.0F, 100.0F));
   update();
}

void RocketGLWidget::uploadMeshes()
{
   meshes.clear();

   for (const RenderItem& item : items)
   {
      const Mesh& src = item.mesh;
      if (src.vertices.empty() || src.indices.empty())
         continue;

      auto gm = std::make_unique<GpuMesh>();

      gm->vao.create();
      gm->vao.bind();

      gm->vbo.create();
      gm->vbo.bind();
      gm->vbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
      gm->vbo.allocate(src.vertices.data(),
                       static_cast<int>(src.vertices.size() * sizeof(Vertex)));

      gm->ibo.create();
      gm->ibo.bind();
      gm->ibo.setUsagePattern(QOpenGLBuffer::StaticDraw);
      gm->ibo.allocate(src.indices.data(),
                       static_cast<int>(src.indices.size() * sizeof(unsigned int)));

      const int stride = static_cast<int>(sizeof(Vertex));
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride,
                            reinterpret_cast<const void*>(static_cast<std::size_t>(0)));
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                            reinterpret_cast<const void*>(3 * sizeof(float)));

      gm->vao.release();
      gm->vbo.release();
      gm->ibo.release();

      gm->indexCount      = static_cast<int>(src.indices.size());
      gm->typeName        = item.typeName;
      gm->overlapOffender = item.overlapOffender;

      // Wireframe geometry: expand each triangle into its three edges as an explicit GL_LINES
      // buffer (positions + normals preserved so the lit shader shades the lines identically). This
      // replaces glPolygonMode(GL_LINE), which does not exist on GLES2 / the generic functions.
      std::vector<Vertex> wireVerts;
      wireVerts.reserve(src.indices.size() * 2U);
      for (std::size_t i = 0; i + 2U < src.indices.size(); i += 3U)
      {
         const Vertex& a = src.vertices[src.indices[i]];
         const Vertex& b = src.vertices[src.indices[i + 1U]];
         const Vertex& c = src.vertices[src.indices[i + 2U]];
         wireVerts.insert(wireVerts.end(), {a, b, b, c, c, a});
      }

      gm->wireVao.create();
      gm->wireVao.bind();
      gm->wireVbo.create();
      gm->wireVbo.bind();
      gm->wireVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
      gm->wireVbo.allocate(wireVerts.data(),
                           static_cast<int>(wireVerts.size() * sizeof(Vertex)));
      glEnableVertexAttribArray(0);
      glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride,
                            reinterpret_cast<const void*>(static_cast<std::size_t>(0)));
      glEnableVertexAttribArray(1);
      glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                            reinterpret_cast<const void*>(3 * sizeof(float)));
      gm->wireVao.release();
      gm->wireVbo.release();
      gm->wireVertexCount = static_cast<int>(wireVerts.size());

      meshes.push_back(std::move(gm));
   }

   applySchemeColors();
}

void RocketGLWidget::buildOverlays()
{
   // --- ground grid in the world X-Z plane (rocket is stood up to +Y) ---
   const float r       = std::max(bounds.radius(), 0.1F);
   const float extent  = r * 2.5F;
   const int   divs    = 20;
   const float step    = (extent * 2.0F) / static_cast<float>(divs);

   // Sit the grid at the rocket's base. World up (+Y) is body-frame +z, so the lowest point of the
   // stood-up rocket is its minimum body-frame z; fall back to the origin when there is no geometry.
   const float groundY = bounds.valid ? bounds.min.z() : 0.0F;

   std::vector<float> gridData;
   gridData.reserve(static_cast<std::size_t>((divs + 1) * 4 * 3));
   for (int i = 0; i <= divs; ++i)
   {
      const float t = -extent + step * static_cast<float>(i);
      // Lines parallel to Z.
      gridData.insert(gridData.end(), {t, groundY, -extent});
      gridData.insert(gridData.end(), {t, groundY, extent});
      // Lines parallel to X.
      gridData.insert(gridData.end(), {-extent, groundY, t});
      gridData.insert(gridData.end(), {extent, groundY, t});
   }

   if (!gridVao.isCreated())
      gridVao.create();
   gridVao.bind();
   if (!gridVbo.isCreated())
      gridVbo.create();
   gridVbo.bind();
   gridVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
   gridVbo.allocate(gridData.data(), static_cast<int>(gridData.size() * sizeof(float)));
   glEnableVertexAttribArray(0);
   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * static_cast<int>(sizeof(float)),
                         reinterpret_cast<const void*>(static_cast<std::size_t>(0)));
   gridVao.release();
   gridVbo.release();
   gridVertexCount = static_cast<int>(gridData.size() / 3);

   // --- XYZ axis triad (one segment per axis; colored per draw in paintGL) ---
   const float axisLen = extent;
   const float axes[]  = {
      0.0F, 0.0F, 0.0F, axisLen, 0.0F, 0.0F, // X
      0.0F, 0.0F, 0.0F, 0.0F, axisLen, 0.0F, // Y
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, axisLen  // Z
   };

   if (!axesVao.isCreated())
      axesVao.create();
   axesVao.bind();
   if (!axesVbo.isCreated())
      axesVbo.create();
   axesVbo.bind();
   axesVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
   axesVbo.allocate(axes, static_cast<int>(sizeof(axes)));
   glEnableVertexAttribArray(0);
   glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * static_cast<int>(sizeof(float)),
                         reinterpret_cast<const void*>(static_cast<std::size_t>(0)));
   axesVao.release();
   axesVbo.release();
   axesVertexCount = 6;
}

void RocketGLWidget::applySchemeColors()
{
   // A part the diagnostics sweep flagged as an overlap offender renders in a fixed error red, overriding
   // its type color, so a self-intersecting design is unmistakable regardless of the active scheme.
   static const QVector3D kErrorColor{0.90F, 0.10F, 0.10F};
   for (const auto& meshPtr : meshes)
      meshPtr->color = meshPtr->overlapOffender ? kErrorColor
                                                : qcolorToVec3(scheme.colorFor(meshPtr->typeName));
}

void RocketGLWidget::cleanup()
{
   for (const auto& meshPtr : meshes)
   {
      if (meshPtr->vbo.isCreated())
         meshPtr->vbo.destroy();
      if (meshPtr->ibo.isCreated())
         meshPtr->ibo.destroy();
      if (meshPtr->vao.isCreated())
         meshPtr->vao.destroy();
      if (meshPtr->wireVbo.isCreated())
         meshPtr->wireVbo.destroy();
      if (meshPtr->wireVao.isCreated())
         meshPtr->wireVao.destroy();
   }
   meshes.clear();

   if (gridVbo.isCreated())
      gridVbo.destroy();
   if (gridVao.isCreated())
      gridVao.destroy();
   gridVertexCount = 0;

   if (axesVbo.isCreated())
      axesVbo.destroy();
   if (axesVao.isCreated())
      axesVao.destroy();
   axesVertexCount = 0;

   litProgram.reset();
   lineProgram.reset();
}

QMatrix4x4 RocketGLWidget::viewProjection() const
{
   const float w      = static_cast<float>(width());
   const float h      = static_cast<float>(std::max(height(), 1));
   const float aspect = w / h;

   const float radius = bounds.radius();
   const float nearP  = std::max(radius * 0.01F, 1e-3F);
   const float farP   = radius * 20.0F + 10.0F;

   QMatrix4x4 proj;
   proj.perspective(45.0F, aspect, nearP, farP);

   const float yawR   = qDegreesToRadians(camYaw);
   const float pitchR = qDegreesToRadians(camPitch);
   const QVector3D dir(std::cos(pitchR) * std::sin(yawR),
                       std::sin(pitchR),
                       std::cos(pitchR) * std::cos(yawR));
   const QVector3D eye = camTarget + dir * camDistance;

   QMatrix4x4 view;
   view.lookAt(eye, camTarget, QVector3D(0.0F, 1.0F, 0.0F));

   return proj * view;
}

} // namespace viz
