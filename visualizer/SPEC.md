# QtRocket Visualizer — implementation spec (shared brain for all agents)

Standalone Qt6 + OpenGL utility that opens a QtRocket design file (`.qrd`) and renders the rocket
as interactive, user-colorable 3D geometry. Greenfield; lives entirely in `visualizer/`. The
**headers, CMakeLists.txt, and this file are FROZEN** — implement the `.cpp` bodies (and `main.cpp`,
`README.md`) against them exactly. Do **not** change the public signatures in the `.h` files.

## Hard constraints (the build fails otherwise)

- The project compiles with `-Wall -Wextra -Wpedantic -Werror` (clang/gcc). **Zero warnings.**
  - No unused parameters/variables (use `[[maybe_unused]]` or omit the name).
  - No signed/unsigned comparison mismatches (indices are `unsigned int`; loop counters likewise, or
    cast deliberately). `.size()` is `size_t`.
  - Initialize everything; no narrowing in brace-init.
  - No C++ comments that trigger `-Wpedantic` (e.g. avoid `//` continued with backslash).
- C++23, SI units (meters, kg, seconds). Match the surrounding house style: include-guard macros
  (already in the headers), Doxygen-ish `///`/`/** */` comments, 3-space-ish indentation as seen in
  the repo (follow each header's existing indentation).
- Modern programmable pipeline (VAOs/VBOs/shaders), **no fixed-function/legacy GL**, but written
  against the version-agnostic GL/GLES2 common subset rather than a forced 3.3 core profile. The
  default `QSurfaceFormat` requests no specific version/profile (see `main.cpp`) so Qt can create
  whatever the platform supports — a desktop compatibility context or a GLES2 context — which is
  what keeps it working across drivers (forcing 3.3 core fails with `EGL_BAD_MATCH` on EGL/NVIDIA).
  Shaders ship in GLSL 1.20 and GLSL ES 1.00 twins, chosen at runtime via
  `QOpenGLContext::isOpenGLES()`. Because there is no `glPolygonMode` on GLES2, wireframe is drawn
  as explicit `GL_LINES` edge geometry, not a polygon-mode toggle.
- Use Qt's GL wrappers: `QOpenGLFunctions` (the generic common-subset class — the widget inherits
  it; call GL through `this`/the inherited members), `QOpenGLShaderProgram`, `QOpenGLBuffer`,
  `QOpenGLVertexArrayObject`, `QMatrix4x4`, `QVector3D`.

## Geometry conventions (CRITICAL — get these exactly right)

The rocket body frame: **z is the longitudinal axis, +z = forward (nose tip), −z = aft.** Ground/up
is handled by the renderer (it stands the rocket up); the mesh builders work purely in body-frame z.

Parts form a tree (`model::part::Part`). `getChildParts()` returns `(shared_ptr<Part>, Vector3
offset)` pairs, where `offset` is the child's center **relative to the parent's center**. The root's
center is the origin. Designs are authored so these offsets are **geometric-center to
geometric-center** (verified: in `mid29_basic.qrd` the nose→body offset is `−0.275 = −(0.15/2 +
0.40/2)`). Therefore the visualizer places each part's **geometric center** at its accumulated
station = sum of offsets from the root down. With each primitive built centered on the origin, a
part is positioned by translating its mesh by that accumulated `Vector3`. This makes parts join
cleanly (e.g. the nose base meets the body top; the fin root trailing edge sits at the body tail).

Read each part's geometry by `dynamic_cast` to the concrete type (include the concrete headers):

- **NoseCone** = `model::part::ConicalNoseCone` — `getBaseRadius()`, `getLength()` (also
  `isSolid()`, `getWallThickness()` if you want a hollow shell look — optional). Render as a cone:
  **tip at `+length/2`, base (radius `baseRadius`) at `−length/2`.** `buildCone(baseRadius, length)`.
- **BodyTube** = `model::part::BodyTube` — `getInnerRadius()`, `getOuterRadius()`, `getLength()`.
  Render as a hollow cylinder centered on origin, spanning `[−L/2, +L/2]`. `buildTube(ri, ro, L)`.
- **FinSet** = `model::part::FinSet` — `getFinCount()`, `getRootChord()`, `getTipChord()`,
  `getSpan()`, `getSweep()`, `getThickness()`, `getBodyRadius()`. `buildFinSet(...)`.
- **HollowSphere** = `model::part::HollowSphere` — `getOuterRadius()`. Render as a sphere
  (`buildSphere(outerRadius)`). This is the default placeholder body; rarely in real designs but
  handle it so an empty/boot design still shows something.
- **Motor**: not present in the loaded tree (the serializer stores it by name and we load against an
  EMPTY motor DB), so you will not encounter it. If a `Motor` ever appears, skip it.

Unknown/other types: skip gracefully (don't crash).

### Primitive local frames (what each builder must produce)

All centered on the origin; `radialSegments` (default 64) = tessellation around z.

- `buildCone(R, L)`: lateral cone surface from base ring (radius `R`, `z=−L/2`) to tip (`z=+L/2`),
  plus a base cap disc at `z=−L/2` (normal `−z`). Lateral **outward** normal at azimuth φ is
  `normalize(L·cosφ, L·sinφ, R)` (derive: slant tangent base→tip is `(−R, L)` in (radial, z); the
  outward normal is `(L, R)`, normalized, then spun to azimuth φ). Smooth around the axis.
- `buildTube(ri, ro, L)`: outer wall (radius `ro`, outward radial normals), inner wall (radius `ri`,
  **inward** radial normals), and two annular end caps at `z=±L/2` (normals `±z`). If `ri == 0`
  (or `< ~1e-9`): omit the inner wall and make the caps full discs (it's a solid rod).
- `buildSphere(r, rings, sectors)`: standard UV sphere, outward normals.
- `buildFinSet(N, cr, ct, s, sweep, thk, rb)`: `N` identical trapezoidal flat plates spaced evenly
  by `2π/N` around z, each a thin prism extruded `±thk/2` off its radial plane. In a fin's local
  plane (radial axis = +x, axial = z), the **root chord is centered on z=0**:
  - root LE: `(x=rb,    z=+cr/2)`,  root TE: `(x=rb,    z=−cr/2)`
  - tip  LE: `(x=rb+s,  z=+cr/2 − sweep)`,  tip TE: `(x=rb+s, z=+cr/2 − sweep − ct)`
  Build the 4-vertex trapezoid, extrude by thickness → a closed prism (2 trapezoid faces + 4 edge
  quads), **flat normals per face** (duplicate verts per face; cross-product winding outward).
  Then rotate the whole prism about z by `i·2π/N` for `i in [0, N)`. (`getTipChord()` may be 0 →
  a triangular fin: the two tip vertices coincide; that's fine, just don't emit degenerate tris if
  you can avoid it, or keep them — harmless.)

`Mesh::append(other)` merges `other` into `this`, offsetting `other`'s indices by the current vertex
count. `buildRocketMeshes(rocket)` returns one `RenderItem` per part (its positioned `Mesh`,
`typeName`, `name`); recurse depth-first accumulating the `Vector3` station. `computeBounds(items)`
is the AABB over all vertices (set `valid=false` if there are no vertices).

`Vector3` is Eigen (`utils/math/MathTypes.h`): `.x() .y() .z()`. Translate a vertex by adding the
station components to `px,py,pz` (normals unchanged by translation).

## Rendering (`RocketGLWidget.cpp`)

- `initializeGL()`: `initializeOpenGLFunctions()`; set clear color from `scheme.background`; enable
  `GL_DEPTH_TEST`; (optionally `GL_CULL_FACE` — but fins are double-sided thin plates and tube
  interiors are visible, so prefer **culling OFF** for correctness, or be careful). Compile the two
  shader programs. Set `glReady=true`, then `uploadMeshes()` (if items pending) + `buildOverlays()`.
  Emit `rendererInfo((const char*)glGetString(GL_RENDERER), (const char*)glGetString(GL_VERSION))`.
- **Lit shader** (`litProgram`): vertex in `layout(location=0) vec3 aPos; location=1 vec3 aNormal;`
  uniforms `mat4 uMvp; mat4 uModel; vec3 uColor; vec3 uLightDir; ...`. Lambert diffuse + a healthy
  ambient term (≈0.35) + optional simple specular, single directional "headlight". Since the model
  transform here is rotation-only (uniform), transforming the normal by `mat3(uModel)` is fine.
- **Line shader** (`lineProgram`): `location=0 vec3 aPos;` uniforms `mat4 uMvp; vec3 uColor;` —
  unlit, for the grid and axes.
- `paintGL()`: clear; compute `mvp = viewProjection() * model` where `model` is the **stand-up**
  rotation that maps body +z to world up (+Y): rotate −90° about X (`QMatrix4x4::rotate(-90, 1,0,0)`).
  Draw grid (if `showGrid`) and axes (if `showAxes`) with `lineProgram` (axes ignore depth-test or
  draw last—your call; keep it simple). Set polygon mode to `GL_LINE` if `wireframe` else `GL_FILL`
  for the rocket. For each `GpuMesh`: bind program, set `uMvp`, `uModel`, `uColor=color`, bind VAO,
  `glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0)`.
- **Camera** (orbit): `viewProjection()` builds a perspective (`~45°` fovy, near/far from
  `bounds.radius()`, e.g. near = max(radius*0.01, 1e-3), far = radius*20 + 10) times a look-at. Eye
  = `camTarget + camDistance * dir(camYaw, camPitch)` with world up = +Y. Map screen drags to
  `camYaw/camPitch` (clamp pitch to ≈[−89,89]); right/middle-drag pans `camTarget` in the camera
  plane scaled by distance; wheel scales `camDistance` (clamp to a sane min/max). Call `update()`.
- `setRenderItems(items)`: store `items`, recompute `bounds`; if `glReady` → `makeCurrent();
  uploadMeshes(); doneCurrent();` then `resetCamera()` + `update()`. If not ready yet, defer (the
  upload happens at the end of `initializeGL`). `uploadMeshes()` clears old `meshes`, then per item
  creates a `GpuMesh` (VAO+VBO+IBO), uploads interleaved `Vertex` data (stride
  `sizeof(viz::Vertex)`, position offset 0, normal offset `3*sizeof(float)`), sets `indexCount`,
  `typeName`; finally `applySchemeColors()`.
- `resetCamera()`: `camTarget = bounds.center()`, `camDistance = max(bounds.radius()*2.5, 0.05)`,
  pleasant default `camYaw/camPitch`, `update()`.
- `setColorScheme(scheme)`: store, `applySchemeColors()`, update clear color (needs current context
  for the clear color — or just set it in `paintGL` each frame from `scheme.background`), `update()`.
- `applySchemeColors()`: for each `GpuMesh`, `color = qcolorToVec3(scheme.colorFor(typeName))`.
- Lifetime: create GL objects only with a current context. In the destructor call `cleanup()` after
  `makeCurrent()`. `cleanup()` destroys VAOs/VBOs and resets programs; idempotent.

Convert `QColor`→`QVector3D` via `redF()/greenF()/blueF()`.

## Window / UI (`VisualizerWindow.cpp`, `main.cpp`)

- `VisualizerWindow`: central widget = `RocketGLWidget`; a right-hand panel (dock or a side widget)
  with: a **scheme preset** `QComboBox` (filled from `viz::presets()`), one **per-type color
  button** per known type (swatch shows the current color; click → `QColorDialog`), a **Reset
  colors** button, and toggle checkboxes for grid / axes / wireframe wired to the widget's slots.
  Menu: **File ▸ Open…** (`onOpen` → `QFileDialog::getOpenFileName(..., "QtRocket designs (*.qrd)")`)
  and **File ▸ Quit**. A status bar showing the file name + part count + the GL renderer string
  (connect `RocketGLWidget::rendererInfo`).
- `openFile(path)`: `DesignSerializer::load(*rocket, *motors, path.toStdString())` inside try/catch
  (show `QMessageBox::warning` on failure, return false). On success: `refreshView()`, update
  `currentFile` + status, return true. `rocket`/`motors` are constructed in the ctor
  (`std::make_unique<model::RocketModel>()`, `...MotorModelDatabase>()`).
- `refreshView()`: `glWidget->setRenderItems(viz::buildRocketMeshes(*rocket))`.
- Color buttons keyed by typeName in `colorButtons`; `rebuildColorButtons()` builds them for the
  scheme's known types (use the union of the default scheme's types so all of NoseCone/BodyTube/
  FinSet/HollowSphere always appear). `onPickColor()` uses `sender()`/a property to know which type.
- `main.cpp`: set `qputenv("QT_WIDGETS_RHI", "0")` (so the widget compositor doesn't try to create
  its own GLES2 RHI backing-store context) and a deliberately minimal `QSurfaceFormat` — depth
  buffer 24, but NO version, NO profile, and NO multisample request — then
  `QSurfaceFormat::setDefaultFormat(...)` **before** creating the `QApplication`/window. Forcing a
  version/profile (or MSAA) here makes context creation fail with `EGL_BAD_MATCH` (0x3009) on
  EGL/NVIDIA paths while still working on GLX, i.e. it runs on one machine but goes black on
  another; leaving it unconstrained lets Qt pick a context the platform can actually create. Then
  construct `VisualizerWindow`, `resize(~1200x800)`, `show()`; if `argc>1` treat `argv[1]` as a
  `.qrd` path and `openFile` it; `return app.exec();`.

## Default color scheme ("Classic", `presets().front()`)

Sane, readable defaults (tweak slightly if you like, keep good contrast on the dark background):
- NoseCone → crimson red `#C0392B`
- BodyTube → light silver `#ECF0F1`
- FinSet → cobalt blue `#2980B9`
- HollowSphere → grey `#95A5A6`
- defaultColor → `#A0A0A0`, background → dark slate `#2B303B`

Provide ≥3 more presets, e.g. **Hi-Vis** (orange nose `#E67E22`, white body, near-black fins,
lighter bg), **Carbon** (dark greys + one accent, near-black bg), **Pastel** (soft tints). Each
preset must map all four known types. `colorFor` falls back to `defaultColor`; `setColor` inserts/
overwrites.

## Sample files for manual testing

`tests/data/designs/*.qrd` — e.g. `mid29_basic.qrd` (nose+body+3 fins), `xl75_multi.qrd`
(nose+body+6 fins+coupler), `small18_shell.qrd` (shell nose). Use these to eyeball the result.
