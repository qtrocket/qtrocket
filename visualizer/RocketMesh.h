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

/// @brief One renderable rocket component: its positioned geometry plus the tags the renderer uses
///        to color and label it. The mesh is already placed in the rocket body frame at the part's
///        resolved pose (see buildRocketMeshes), so the renderer needs only one shared transform.
struct RenderItem
{
   Mesh    mesh;
   QString typeName; ///< Part::typeName() -- the ColorScheme lookup key
   QString name;     ///< Part::getName() -- for UI / picking / tooltips

   /// An overlap offender per the diagnostics sweep: the renderer overrides its type color with an
   /// error color; @ref overlapMessage carries the located reason for a tooltip / status line.
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
// Convention: z is the longitudinal axis (+z = forward/nose). Each primitive is built in local
// coordinates centered on the origin, so buildRocketMeshes can position a part by a pure
// translation of its resolved pose. radialSegments controls the tessellation around z.

/// @brief Solid right circular cone: tip at +length/2, base (radius @p baseRadius) at -length/2.
Mesh buildCone(double baseRadius, double length, int radialSegments = 64);

/// @brief Hollow cylinder spanning z in [-length/2, +length/2]: outer wall (@p outerRadius), inner
///        wall (@p innerRadius), and the two annular end caps. @p innerRadius == 0 yields a solid
///        rod (no inner wall; full-disc caps).
Mesh buildTube(double innerRadius, double outerRadius, double length, int radialSegments = 64);

/// @brief UV sphere centered on the origin (used for the placeholder HollowSphere body).
Mesh buildSphere(double radius, int rings = 24, int sectors = 48);

/// @brief @p finCount identical trapezoidal flat-plate fins arrayed evenly around z, mounted at
///        @p bodyRadius and extruded by @p thickness. The root chord is centered on z = 0 (LE at
///        +rootChord/2, TE at -rootChord/2); @p sweep shifts the tip aft.
Mesh buildFinSet(unsigned int finCount, double rootChord, double tipChord, double span,
                 double sweep, double thickness, double bodyRadius);

// ---- Tree walk --------------------------------------------------------------------------------

/// @brief Convert a loaded rocket's part tree into a flat list of positioned RenderItems.
///        Resolves placements once via model::part::resolvePlacements -- the same poses the
///        simulator consumes, so the two can't diverge -- then translates each origin-centered
///        primitive so its center lands half an axialLength aft of the resolved fore-plane origin.
///        Concrete part types are dispatched by dynamic_cast to read their geometry.
std::vector<RenderItem> buildRocketMeshes(const model::RocketModel& rocket);

/// @brief Axis-aligned bounds over @p items (invalid Bounds for an empty list).
Bounds computeBounds(const std::vector<RenderItem>& items);

} // namespace viz

#endif // VISUALIZER_ROCKETMESH_H
