#ifndef VISUALIZER_ROCKETMESH_H
#define VISUALIZER_ROCKETMESH_H

// Qt headers
#include <QString>
#include <QVector3D>

// C++ headers
#include <vector>

// Borrowed handle: buildRocketMeshes walks a loaded rocket's part tree. The full type is only
// needed in the .cpp, which includes model/RocketModel.h.
namespace model { class RocketModel; }

namespace viz
{

/**
 * @brief One interleaved vertex: position (meters) + normal (unit). Matches the GL vertex layout
 *        consumed by RocketGLWidget (two vec3 attributes, location 0 = position, 1 = normal).
 */
struct Vertex
{
   float px{0.0F}, py{0.0F}, pz{0.0F}; ///< position (m)
   float nx{0.0F}, ny{0.0F}, nz{0.0F}; ///< unit normal
};

/**
 * @brief A CPU-side triangle mesh: interleaved vertices + a flat index list (3 indices per
 *        triangle). Geometry is in the rocket body frame, SI meters, +z = forward (nose).
 */
struct Mesh
{
   std::vector<Vertex>       vertices;
   std::vector<unsigned int> indices;

   /// @brief Append @p other into this mesh, rebasing its indices by the current vertex count.
   void append(const Mesh& other);
};

/**
 * @brief One renderable rocket component: its positioned geometry plus the tags the renderer uses
 *        to color and label it. The mesh is already placed in the rocket body frame (the part's
 *        geometric center sits at its accumulated CM-to-CM position -- see buildRocketMeshes), so
 *        the renderer needs only a single shared transform.
 */
struct RenderItem
{
   Mesh    mesh;
   QString typeName; ///< Part::typeName() -- the ColorScheme lookup key
   QString name;     ///< Part::getName() -- for UI / picking / tooltips

   /// @brief True when the diagnostics sweep flagged this part as an overlap offender. The renderer
   ///        overrides the type color with an error color so a self-intersecting design reads at a
   ///        glance; @ref overlapMessage carries the located reason for a tooltip / status line.
   bool    overlapOffender{false};
   QString overlapMessage; ///< located diagnostic for this offender (empty when not an offender)
};

/**
 * @brief Axis-aligned bounding box over a set of RenderItems, used to auto-frame the camera.
 */
struct Bounds
{
   QVector3D min;
   QVector3D max;
   bool      valid{false}; ///< false when computed over an empty geometry set

   QVector3D center() const { return (min + max) * 0.5F; }
   float     radius() const { return valid ? (max - min).length() * 0.5F : 1.0F; } ///< half-diagonal
};

// ---- Primitive builders -----------------------------------------------------------------------
// Convention: z is the longitudinal axis (+z = forward/nose). Each primitive is built in LOCAL
// coordinates centered on the origin, per the placement rules in SPEC.md, so buildRocketMeshes can
// position a part by a pure translation of its accumulated CM-to-CM station onto the geometric
// center. radialSegments controls the tessellation around the z axis.

/// @brief Solid right circular cone: tip at +length/2, base (radius @p baseRadius) at -length/2.
Mesh buildCone(double baseRadius, double length, int radialSegments = 64);

/// @brief Hollow cylinder spanning z in [-length/2, +length/2]: outer wall (@p outerRadius), inner
///        wall (@p innerRadius), and the two annular end caps. @p innerRadius == 0 yields a solid
///        rod (no inner wall; full-disc caps).
Mesh buildTube(double innerRadius, double outerRadius, double length, int radialSegments = 64);

/// @brief UV sphere centered on the origin (used for the placeholder HollowSphere body).
Mesh buildSphere(double radius, int rings = 24, int sectors = 48);

/// @brief @p finCount identical trapezoidal flat-plate fins arrayed evenly around the z axis,
///        mounted at @p bodyRadius and extruded by @p thickness. The ROOT chord is centered on
///        z = 0 (root LE at +rootChord/2, root TE at -rootChord/2); @p sweep shifts the tip aft.
Mesh buildFinSet(unsigned int finCount, double rootChord, double tipChord, double span,
                 double sweep, double thickness, double bodyRadius);

// ---- Tree walk --------------------------------------------------------------------------------

/**
 * @brief Convert a loaded rocket's part tree into a flat list of positioned RenderItems.
 *
 * Walks @p rocket.getTopPart() depth-first. Each part's accumulated body-frame station is the sum
 * of the CM-to-CM @c offset values from the root down to it; the part's geometry (centered on the
 * origin by the builders above) is translated so its geometric center lands on that station. This
 * reproduces the design's authored "geometric-center to geometric-center" stacking, so parts join
 * cleanly. Each concrete part type is dispatched by typeName() / dynamic_cast to read its geometry.
 */
std::vector<RenderItem> buildRocketMeshes(const model::RocketModel& rocket);

/// @brief Axis-aligned bounds over @p items (invalid Bounds for an empty list).
Bounds computeBounds(const std::vector<RenderItem>& items);

} // namespace viz

#endif // VISUALIZER_ROCKETMESH_H
