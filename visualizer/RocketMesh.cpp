// Matching header
#include "visualizer/RocketMesh.h"

// qtrocket headers
#include "model/RocketModel.h"
#include "model/parts/Part.h"
#include "model/parts/ConicalNoseCone.h"
#include "model/parts/BodyTube.h"
#include "model/parts/FinSet.h"
#include "model/parts/HollowSphere.h"
#include "utils/Logger.h"
#include "utils/math/MathTypes.h"

// C++ headers
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <tuple>

namespace viz
{

namespace
{

/// @brief 2*pi as a double, for spinning a profile around the longitudinal (z) axis.
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

/// @brief Make a Vertex from doubles, narrowing to float explicitly so brace-init never trips
///        -Wnarrowing.
Vertex makeVertex(double px, double py, double pz, double nx, double ny, double nz)
{
   return Vertex{static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz),
                 static_cast<float>(nx), static_cast<float>(ny), static_cast<float>(nz)};
}

/// @brief Append a triangle (three already-pushed vertex indices) to a mesh's index list.
void addTriangle(Mesh& m, unsigned int a, unsigned int b, unsigned int c)
{
   m.indices.push_back(a);
   m.indices.push_back(b);
   m.indices.push_back(c);
}

/// @brief Push a flat-shaded triangle: three positions sharing one face normal from the edge cross
///        product (CCW winding => outward normal). Drops degenerate (zero-area) triangles so no NaN
///        normals leak.
void addFlatTriangle(Mesh& m, const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
   const Vector3 edge1 = p1 - p0;
   const Vector3 edge2 = p2 - p0;
   Vector3 normal = edge1.cross(edge2);
   const double len = normal.norm();
   if(len < 1e-12)
   {
      return; // degenerate triangle (e.g. a collapsed tip edge): drop it rather than emit NaNs
   }
   normal /= len;

   const unsigned int base = static_cast<unsigned int>(m.vertices.size());
   m.vertices.push_back(makeVertex(p0.x(), p0.y(), p0.z(), normal.x(), normal.y(), normal.z()));
   m.vertices.push_back(makeVertex(p1.x(), p1.y(), p1.z(), normal.x(), normal.y(), normal.z()));
   m.vertices.push_back(makeVertex(p2.x(), p2.y(), p2.z(), normal.x(), normal.y(), normal.z()));
   addTriangle(m, base, base + 1U, base + 2U);
}

/// @brief Push a flat-shaded quad (p0,p1,p2,p3 wound CCW) as two flat triangles.
void addFlatQuad(Mesh& m, const Vector3& p0, const Vector3& p1, const Vector3& p2,
                 const Vector3& p3)
{
   addFlatTriangle(m, p0, p1, p2);
   addFlatTriangle(m, p0, p2, p3);
}

} // anonymous namespace

void Mesh::append(const Mesh& other)
{
   const unsigned int offset = static_cast<unsigned int>(vertices.size());
   vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
   indices.reserve(indices.size() + other.indices.size());
   for(unsigned int idx : other.indices)
   {
      indices.push_back(idx + offset);
   }
}

Mesh buildCone(double baseRadius, double length, int radialSegments)
{
   Mesh m;
   const int segments = std::max(3, radialSegments);
   const double R = baseRadius;
   const double L = length;
   const double zBase = -L * 0.5;
   const double zTip = L * 0.5;

   // Lateral outward normal direction (in the radial,z plane): (L, R) normalized, then spun to phi.
   const double slantLen = std::sqrt(L * L + R * R);
   const double nr = (slantLen > 1e-12) ? (L / slantLen) : 1.0; // radial component of the lateral normal
   const double nz = (slantLen > 1e-12) ? (R / slantLen) : 0.0; // axial component of the lateral normal

   // Smooth lateral surface: one base ring + a tip ring (tip vertices share the position but carry
   // the per-azimuth lateral normal so the cone shades smoothly around the axis).
   const unsigned int baseRingStart = static_cast<unsigned int>(m.vertices.size());
   for(int i = 0; i <= segments; ++i)
   {
      const double phi = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
      const double c = std::cos(phi);
      const double s = std::sin(phi);
      m.vertices.push_back(makeVertex(R * c, R * s, zBase, nr * c, nr * s, nz));
   }
   const unsigned int tipRingStart = static_cast<unsigned int>(m.vertices.size());
   for(int i = 0; i <= segments; ++i)
   {
      const double phi = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
      const double c = std::cos(phi);
      const double s = std::sin(phi);
      m.vertices.push_back(makeVertex(0.0, 0.0, zTip, nr * c, nr * s, nz));
   }
   for(int i = 0; i < segments; ++i)
   {
      const unsigned int b0 = baseRingStart + static_cast<unsigned int>(i);
      const unsigned int b1 = baseRingStart + static_cast<unsigned int>(i) + 1U;
      const unsigned int t0 = tipRingStart + static_cast<unsigned int>(i);
      const unsigned int t1 = tipRingStart + static_cast<unsigned int>(i) + 1U;
      // CCW as seen from outside: base->base->tip
      addTriangle(m, b0, b1, t1);
      addTriangle(m, b0, t1, t0);
   }

   // Base cap disc at z = zBase, normal -z (outward = aft). Fan around a center vertex.
   const unsigned int capCenter = static_cast<unsigned int>(m.vertices.size());
   m.vertices.push_back(makeVertex(0.0, 0.0, zBase, 0.0, 0.0, -1.0));
   const unsigned int capRingStart = static_cast<unsigned int>(m.vertices.size());
   for(int i = 0; i <= segments; ++i)
   {
      const double phi = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
      m.vertices.push_back(makeVertex(R * std::cos(phi), R * std::sin(phi), zBase, 0.0, 0.0, -1.0));
   }
   for(int i = 0; i < segments; ++i)
   {
      const unsigned int r0 = capRingStart + static_cast<unsigned int>(i);
      const unsigned int r1 = capRingStart + static_cast<unsigned int>(i) + 1U;
      // wound so the normal faces -z (outward from the cap, viewed from below)
      addTriangle(m, capCenter, r1, r0);
   }

   return m;
}

Mesh buildTube(double innerRadius, double outerRadius, double length, int radialSegments)
{
   Mesh m;
   const int segments = std::max(3, radialSegments);
   const double ri = innerRadius;
   const double ro = outerRadius;
   const double zTop = length * 0.5;  // forward (+z)
   const double zBot = -length * 0.5; // aft (-z)
   const bool solid = (ri < 1e-9);

   // ---- Outer wall (outward radial normals) --------------------------------------------------
   {
      const unsigned int topStart = static_cast<unsigned int>(m.vertices.size());
      for(int i = 0; i <= segments; ++i)
      {
         const double phi = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
         const double c = std::cos(phi);
         const double s = std::sin(phi);
         m.vertices.push_back(makeVertex(ro * c, ro * s, zTop, c, s, 0.0));
      }
      const unsigned int botStart = static_cast<unsigned int>(m.vertices.size());
      for(int i = 0; i <= segments; ++i)
      {
         const double phi = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
         const double c = std::cos(phi);
         const double s = std::sin(phi);
         m.vertices.push_back(makeVertex(ro * c, ro * s, zBot, c, s, 0.0));
      }
      for(int i = 0; i < segments; ++i)
      {
         const unsigned int tt0 = topStart + static_cast<unsigned int>(i);
         const unsigned int tt1 = topStart + static_cast<unsigned int>(i) + 1U;
         const unsigned int bb0 = botStart + static_cast<unsigned int>(i);
         const unsigned int bb1 = botStart + static_cast<unsigned int>(i) + 1U;
         addTriangle(m, bb0, bb1, tt1);
         addTriangle(m, bb0, tt1, tt0);
      }
   }

   // ---- Inner wall (inward radial normals) ---------------------------------------------------
   if(!solid)
   {
      const unsigned int topStart = static_cast<unsigned int>(m.vertices.size());
      for(int i = 0; i <= segments; ++i)
      {
         const double phi = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
         const double c = std::cos(phi);
         const double s = std::sin(phi);
         m.vertices.push_back(makeVertex(ri * c, ri * s, zTop, -c, -s, 0.0));
      }
      const unsigned int botStart = static_cast<unsigned int>(m.vertices.size());
      for(int i = 0; i <= segments; ++i)
      {
         const double phi = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
         const double c = std::cos(phi);
         const double s = std::sin(phi);
         m.vertices.push_back(makeVertex(ri * c, ri * s, zBot, -c, -s, 0.0));
      }
      for(int i = 0; i < segments; ++i)
      {
         const unsigned int tt0 = topStart + static_cast<unsigned int>(i);
         const unsigned int tt1 = topStart + static_cast<unsigned int>(i) + 1U;
         const unsigned int bb0 = botStart + static_cast<unsigned int>(i);
         const unsigned int bb1 = botStart + static_cast<unsigned int>(i) + 1U;
         // reversed winding vs. the outer wall so the visible (inner) face points inward
         addTriangle(m, bb0, tt1, bb1);
         addTriangle(m, bb0, tt0, tt1);
      }
   }

   // ---- End caps -----------------------------------------------------------------------------
   // Helper lambda to emit one annular (or full-disc when solid) cap at z = zc, with normal +/-z.
   auto emitCap = [&](double zc, double nzSign)
   {
      if(solid)
      {
         const unsigned int center = static_cast<unsigned int>(m.vertices.size());
         m.vertices.push_back(makeVertex(0.0, 0.0, zc, 0.0, 0.0, nzSign));
         const unsigned int ringStart = static_cast<unsigned int>(m.vertices.size());
         for(int i = 0; i <= segments; ++i)
         {
            const double phi = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
            m.vertices.push_back(
               makeVertex(ro * std::cos(phi), ro * std::sin(phi), zc, 0.0, 0.0, nzSign));
         }
         for(int i = 0; i < segments; ++i)
         {
            const unsigned int r0 = ringStart + static_cast<unsigned int>(i);
            const unsigned int r1 = ringStart + static_cast<unsigned int>(i) + 1U;
            if(nzSign > 0.0)
            {
               addTriangle(m, center, r0, r1);
            }
            else
            {
               addTriangle(m, center, r1, r0);
            }
         }
      }
      else
      {
         const unsigned int outerStart = static_cast<unsigned int>(m.vertices.size());
         for(int i = 0; i <= segments; ++i)
         {
            const double phi = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
            m.vertices.push_back(
               makeVertex(ro * std::cos(phi), ro * std::sin(phi), zc, 0.0, 0.0, nzSign));
         }
         const unsigned int innerStart = static_cast<unsigned int>(m.vertices.size());
         for(int i = 0; i <= segments; ++i)
         {
            const double phi = kTwoPi * static_cast<double>(i) / static_cast<double>(segments);
            m.vertices.push_back(
               makeVertex(ri * std::cos(phi), ri * std::sin(phi), zc, 0.0, 0.0, nzSign));
         }
         for(int i = 0; i < segments; ++i)
         {
            const unsigned int o0 = outerStart + static_cast<unsigned int>(i);
            const unsigned int o1 = outerStart + static_cast<unsigned int>(i) + 1U;
            const unsigned int in0 = innerStart + static_cast<unsigned int>(i);
            const unsigned int in1 = innerStart + static_cast<unsigned int>(i) + 1U;
            if(nzSign > 0.0)
            {
               addTriangle(m, o0, in0, o1);
               addTriangle(m, o1, in0, in1);
            }
            else
            {
               addTriangle(m, o0, o1, in0);
               addTriangle(m, o1, in1, in0);
            }
         }
      }
   };

   emitCap(zTop, 1.0);
   emitCap(zBot, -1.0);

   return m;
}

Mesh buildSphere(double radius, int rings, int sectors)
{
   Mesh m;
   const int nRings = std::max(2, rings);     // stacks (latitude), from +z pole to -z pole
   const int nSectors = std::max(3, sectors); // slices (longitude)
   const double r = radius;

   for(int i = 0; i <= nRings; ++i)
   {
      // polar angle theta from 0 (+z pole) to pi (-z pole)
      const double theta = 3.14159265358979323846 * static_cast<double>(i)
                           / static_cast<double>(nRings);
      const double sinT = std::sin(theta);
      const double cosT = std::cos(theta);
      for(int j = 0; j <= nSectors; ++j)
      {
         const double phi = kTwoPi * static_cast<double>(j) / static_cast<double>(nSectors);
         const double nx = sinT * std::cos(phi);
         const double ny = sinT * std::sin(phi);
         const double nz = cosT;
         m.vertices.push_back(makeVertex(r * nx, r * ny, r * nz, nx, ny, nz));
      }
   }

   const unsigned int stride = static_cast<unsigned int>(nSectors + 1);
   for(int i = 0; i < nRings; ++i)
   {
      for(int j = 0; j < nSectors; ++j)
      {
         const unsigned int a = static_cast<unsigned int>(i) * stride
                                + static_cast<unsigned int>(j);
         const unsigned int b = a + stride;
         // CCW outward winding
         addTriangle(m, a, a + 1U, b + 1U);
         addTriangle(m, a, b + 1U, b);
      }
   }

   return m;
}

Mesh buildFinSet(unsigned int finCount, double rootChord, double tipChord, double span,
                 double sweep, double thickness, double bodyRadius)
{
   Mesh m;
   const unsigned int n = std::max(1U, finCount);
   const double cr = rootChord;
   const double ct = tipChord;
   const double s = span;
   const double halfThk = thickness * 0.5;
   const double rb = bodyRadius;

   // Fin trapezoid in its local (radial = +x, axial = z) plane, root chord centered on z = 0.
   // Vertices (4-gon): rootLE -> rootTE -> tipTE -> tipLE wound CCW on the +y (+thickness) face.
   const double xRoot = rb;
   const double xTip = rb + s;
   const double zRootLE = cr * 0.5;
   const double zRootTE = -cr * 0.5;
   const double zTipLE = cr * 0.5 - sweep;
   const double zTipTE = cr * 0.5 - sweep - ct;

   // Base prism vertices for one fin in the +x plane, before the per-fin azimuth rotation.
   // Plus side = +thickness/2 (toward +y), minus side = -thickness/2.
   struct V2 { double x; double z; };
   const V2 quad[4] = {
      {xRoot, zRootLE}, // 0 root leading edge
      {xRoot, zRootTE}, // 1 root trailing edge
      {xTip, zTipTE},   // 2 tip trailing edge
      {xTip, zTipLE}    // 3 tip leading edge
   };

   for(unsigned int k = 0; k < n; ++k)
   {
      const double az = kTwoPi * static_cast<double>(k) / static_cast<double>(n);
      const double ca = std::cos(az);
      const double sa = std::sin(az);

      // Rotate a fin-local point (x along radial, y = thickness offset, z axial) about z by az.
      auto place = [&](double x, double y, double z) -> Vector3
      {
         return Vector3(x * ca - y * sa, x * sa + y * ca, z);
      };

      // The two flat trapezoid faces (+y and -y), each a quad with a flat normal.
      Vector3 plus[4];
      Vector3 minus[4];
      for(int i = 0; i < 4; ++i)
      {
         plus[i] = place(quad[i].x, halfThk, quad[i].z);
         minus[i] = place(quad[i].x, -halfThk, quad[i].z);
      }

      // +thickness face: outward normal points toward +y (local), wound CCW seen from +y.
      addFlatQuad(m, plus[0], plus[1], plus[2], plus[3]);
      // -thickness face: outward normal points toward -y, reverse winding.
      addFlatQuad(m, minus[0], minus[3], minus[2], minus[1]);

      // Four edge quads connecting the two faces. Wind each so the normal points away from the
      // fin body. addFlatTriangle drops collapsed edges (e.g. the tip edge when ct == 0).
      // Edge between fin vertices i and i+1: plus[i], plus[i+1], minus[i+1], minus[i].
      for(int i = 0; i < 4; ++i)
      {
         const int j = (i + 1) % 4;
         addFlatQuad(m, plus[i], minus[i], minus[j], plus[j]);
      }
   }

   return m;
}

std::vector<RenderItem> buildRocketMeshes(const model::RocketModel& rocket)
{
   std::vector<RenderItem> items;

   std::shared_ptr<model::part::Part> root = rocket.getTopPart();
   if(!root)
   {
      return items;
   }

   // Translate every vertex of a centered primitive by the part's accumulated body-frame station.
   auto translateMesh = [](Mesh& mesh, const Vector3& station)
   {
      const float dx = static_cast<float>(station.x());
      const float dy = static_cast<float>(station.y());
      const float dz = static_cast<float>(station.z());
      for(Vertex& v : mesh.vertices)
      {
         v.px += dx;
         v.py += dy;
         v.pz += dz;
         // normals unchanged by a pure translation
      }
   };

   // Consume the same absolute placement the simulator does: resolve every part's pose once, then
   // translate each origin-centered primitive so its fore plane lands at the resolved fore-plane
   // origin (the build* primitives are centered about mid-length, half an axialLength forward of
   // the aft plane).
   const std::vector<model::part::Placed> placed =
      model::part::resolvePlacements(*root, model::part::Pose{});

   // Diagnostics verdict (cached per structural resolve): map each offender id to its message so
   // those RenderItems render in an error color. Also log it, so a headless caller sees the same
   // signal the simulator throws on.
   std::map<model::part::PartId, std::string> offenderMessage;
   const model::part::SolveResult& diag = root->placementDiagnostics();
   if(!diag.ok)
   {
      for(const model::part::OverlapDiagnostic& d : diag.diagnostics)
      {
         offenderMessage[d.offender] = d.message;
         utils::Logger::getInstance()->error("RocketMesh: placement overlap -- " + d.message);
      }
   }

   for(const model::part::Placed& p : placed)
   {
      const model::part::Part& part = *p.part;
      Mesh mesh;
      bool hasMesh = false;

      if(const auto* cone = dynamic_cast<const model::part::ConicalNoseCone*>(&part))
      {
         mesh = buildCone(cone->getBaseRadius(), cone->getLength());
         hasMesh = true;
      }
      else if(const auto* tube = dynamic_cast<const model::part::BodyTube*>(&part))
      {
         mesh = buildTube(tube->getInnerRadius(), tube->getOuterRadius(), tube->getLength());
         hasMesh = true;
      }
      else if(const auto* fins = dynamic_cast<const model::part::FinSet*>(&part))
      {
         mesh = buildFinSet(fins->getFinCount(), fins->getRootChord(), fins->getTipChord(),
                            fins->getSpan(), fins->getSweep(), fins->getThickness(),
                            fins->getBodyRadius());
         hasMesh = true;
      }
      else if(const auto* sphere = dynamic_cast<const model::part::HollowSphere*>(&part))
      {
         mesh = buildSphere(sphere->getOuterRadius());
         hasMesh = true;
      }
      // else: unknown/Motor/other -> no geometry.

      if(hasMesh)
      {
         translateMesh(mesh, p.pose.origin + Vector3(0.0, 0.0, -part.axialLength() / 2.0));
         RenderItem item;
         item.mesh = std::move(mesh);
         item.typeName = QString::fromStdString(part.typeName());
         item.name = QString::fromStdString(part.getName());
         if(const auto it = offenderMessage.find(part.getId()); it != offenderMessage.end())
         {
            item.overlapOffender = true;
            item.overlapMessage  = QString::fromStdString(it->second);
         }
         items.push_back(std::move(item));
      }
   }

   return items;
}

Bounds computeBounds(const std::vector<RenderItem>& items)
{
   Bounds b;
   float minX = std::numeric_limits<float>::max();
   float minY = std::numeric_limits<float>::max();
   float minZ = std::numeric_limits<float>::max();
   float maxX = std::numeric_limits<float>::lowest();
   float maxY = std::numeric_limits<float>::lowest();
   float maxZ = std::numeric_limits<float>::lowest();
   bool any = false;

   for(const RenderItem& item : items)
   {
      for(const Vertex& v : item.mesh.vertices)
      {
         any = true;
         minX = std::min(minX, v.px);
         minY = std::min(minY, v.py);
         minZ = std::min(minZ, v.pz);
         maxX = std::max(maxX, v.px);
         maxY = std::max(maxY, v.py);
         maxZ = std::max(maxZ, v.pz);
      }
   }

   if(any)
   {
      b.min = QVector3D(minX, minY, minZ);
      b.max = QVector3D(maxX, maxY, maxZ);
      b.valid = true;
   }

   return b;
}

} // namespace viz
