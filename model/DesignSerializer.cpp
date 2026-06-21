#include "model/DesignSerializer.h"

/// \cond
// C++ headers
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

// 3rd party headers
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
/// \endcond

// qtrocket headers
#include "model/RocketModel.h"
#include "model/MotorModelDatabase.h"
#include "model/parts/Parts.h" // Part, PartParams, makePart, params, Motor, concrete part types
#include "utils/Logger.h"
#include "utils/math/MathTypes.h"

namespace model
{

namespace
{
namespace pt = boost::property_tree;

// ---- write helpers -------------------------------------------------------------------------------

pt::ptree writeParams(const part::PartParams& p)
{
   pt::ptree n;
   auto putd = [&n](const char* key, const std::optional<double>& v)
   { if(v) { n.put(std::string("<xmlattr>.") + key, *v); } };
   putd("innerRadius",   p.innerRadius);
   putd("outerRadius",   p.outerRadius);
   putd("baseRadius",    p.baseRadius);
   putd("length",        p.length);
   putd("wallThickness", p.wallThickness);
   putd("density",       p.density);
   putd("rootChord",     p.rootChord);
   putd("tipChord",      p.tipChord);
   putd("span",          p.span);
   putd("sweep",         p.sweep);
   putd("thickness",     p.thickness);
   putd("bodyRadius",    p.bodyRadius);
   if(p.finCount) { n.put("<xmlattr>.finCount", *p.finCount); }
   if(p.solid)    { n.put("<xmlattr>.solid", *p.solid ? "true" : "false"); } // bool as string, per the DB idiom
   return n;
}

pt::ptree writeOffset(const Vector3& off)
{
   pt::ptree n;
   n.put("<xmlattr>.x", off.x());
   n.put("<xmlattr>.y", off.y());
   n.put("<xmlattr>.z", off.z());
   return n;
}

// Recursively serialize a (non-Motor) part and its non-Motor descendants. @p offset is this part's
// CM-to-CM offset relative to its parent ({0,0,0} for the root, ignored on load).
pt::ptree writePart(const part::Part& node, const Vector3& offset)
{
   pt::ptree pn;
   pn.put("<xmlattr>.type", node.typeName());
   pn.put("<xmlattr>.name", node.getName());
   pn.add_child("params", writeParams(part::params(node)));
   pn.add_child("offset", writeOffset(offset));

   pt::ptree children;
   for(const auto& [child, childOffset] : node.getChildParts())
   {
      // The motor is serialized separately, by common name (see save) -- not as a tree part, since it
      // cannot be rebuilt by the geometry factory.
      if(dynamic_cast<const part::Motor*>(child.get()) != nullptr) { continue; }
      children.add_child("part", writePart(*child, childOffset)); // add_child (NOT put) for siblings
   }
   pn.add_child("children", children);
   return pn;
}

// ---- read helpers --------------------------------------------------------------------------------

template<typename T>
std::optional<T> readOpt(const pt::ptree& node, const std::string& path)
{
   if(const auto v = node.get_optional<T>(path)) { return *v; }
   return std::nullopt;
}

part::PartParams readParams(const pt::ptree& partNode)
{
   part::PartParams p;
   p.name          = partNode.get<std::string>("<xmlattr>.name", "");
   p.innerRadius   = readOpt<double>(partNode, "params.<xmlattr>.innerRadius");
   p.outerRadius   = readOpt<double>(partNode, "params.<xmlattr>.outerRadius");
   p.baseRadius    = readOpt<double>(partNode, "params.<xmlattr>.baseRadius");
   p.length        = readOpt<double>(partNode, "params.<xmlattr>.length");
   p.wallThickness = readOpt<double>(partNode, "params.<xmlattr>.wallThickness");
   p.density       = readOpt<double>(partNode, "params.<xmlattr>.density");
   p.rootChord     = readOpt<double>(partNode, "params.<xmlattr>.rootChord");
   p.tipChord      = readOpt<double>(partNode, "params.<xmlattr>.tipChord");
   p.span          = readOpt<double>(partNode, "params.<xmlattr>.span");
   p.sweep         = readOpt<double>(partNode, "params.<xmlattr>.sweep");
   p.thickness     = readOpt<double>(partNode, "params.<xmlattr>.thickness");
   p.bodyRadius    = readOpt<double>(partNode, "params.<xmlattr>.bodyRadius");
   p.finCount      = readOpt<unsigned int>(partNode, "params.<xmlattr>.finCount");
   if(const auto s = partNode.get_optional<std::string>("params.<xmlattr>.solid"))
   {
      p.solid = (*s == "true");
   }
   return p;
}

std::shared_ptr<part::Part> buildPart(const pt::ptree& partNode)
{
   const std::string type = partNode.get<std::string>("<xmlattr>.type");
   if(type == "Motor")
   {
      // writePart never emits a Motor as a tree part (it skips them); motors round-trip by common
      // name via <motor>. A Motor here means a hand-edited or foreign file -- reject it clearly.
      throw std::runtime_error(
         "DesignSerializer: a Motor cannot be a tree part; it loads by common name via <motor>");
   }
   std::shared_ptr<part::Part> node = part::makePart(type, readParams(partNode)); // throws on bad/unknown

   if(const auto kids = partNode.get_child_optional("children"))
   {
      for(const auto& [key, childNode] : *kids)
      {
         if(key != "part") { continue; } // skip any non-<part> entry (e.g. an <xmlattr> pseudo-node)
         const Vector3 off{ childNode.get<double>("offset.<xmlattr>.x", 0.0),
                            childNode.get<double>("offset.<xmlattr>.y", 0.0),
                            childNode.get<double>("offset.<xmlattr>.z", 0.0) };
         std::shared_ptr<part::Part> child = buildPart(childNode);
         const std::string childName = child->getName();
         const auto before = node->getChildParts().size();
         node->addChildPart(std::move(child), off);
         if(node->getChildParts().size() != before + 1) // addChildPart is a silent no-op on rejection
         {
            throw std::runtime_error(
               "DesignSerializer: failed to attach child part '" + childName + "' (see log for the reason)");
         }
      }
   }
   return node;
}
} // anonymous namespace

void DesignSerializer::save(const RocketModel& rocket, const std::string& filename)
{
   pt::ptree tree;
   // Format version "major.minor": load accepts any minor within major 0 and rejects other majors.
   tree.put("QtRocketDesign.<xmlattr>.version", "0.1");
   tree.put("QtRocketDesign.design.<xmlattr>.name", rocket.getName());

   tree.add_child("QtRocketDesign.part", writePart(*rocket.getTopPart(), Vector3::Zero()));

   if(rocket.isMotorSet())
   {
      tree.put("QtRocketDesign.motor.<xmlattr>.commonName", rocket.getMotorModel().data.commonName);
   }

   // The <sim> block carries only the RocketModel-owned aero options. Launch/environment options
   // (velocity, angle, atmosphere, gravity, integrator) are NOT RocketModel state and are not
   // serialized here -- the CLI manages them as session config (see the spec's §9 follow-ups).
   tree.put("QtRocketDesign.sim.<xmlattr>.dragCoefficient", rocket.getDragCoefficient());
   tree.put("QtRocketDesign.sim.<xmlattr>.referenceArea", rocket.getReferenceArea());
   tree.put("QtRocketDesign.sim.<xmlattr>.referenceAreaOverridden",
            rocket.isReferenceAreaOverridden() ? "true" : "false");

   pt::xml_writer_settings<std::string> settings(' ', 2);
   pt::write_xml(filename, tree, std::locale(), settings);
}

void DesignSerializer::load(RocketModel& rocket, MotorModelDatabase& motors, const std::string& filename)
{
   pt::ptree tree;
   pt::read_xml(filename, tree); // throws on an unreadable / malformed file

   const pt::ptree& root = tree.get_child("QtRocketDesign");
   const std::string version = root.get<std::string>("<xmlattr>.version", "");
   const std::string major = version.substr(0, version.find('.'));
   if(major != "0") // forward-tolerant within major 0; reject an unknown major version
   {
      throw std::runtime_error("DesignSerializer: unsupported design-file version '" + version + "'");
   }

   // Build the whole geometry tree (detached) BEFORE touching the rocket, then install in one shot so
   // a malformed file leaves the existing rocket untouched.
   std::shared_ptr<part::Part> newRoot = buildPart(root.get_child("part"));
   rocket.setRoot(newRoot); // in-place: re-resolves motorPart and resets the ref-area override
   rocket.setName(root.get<std::string>("design.<xmlattr>.name", ""));

   // Apply the sim/aero block AFTER setRoot so a restored manual override wins over setRoot's reset.
   rocket.setDragCoefficient(root.get<double>("sim.<xmlattr>.dragCoefficient", rocket.getDragCoefficient()));
   const bool overridden =
      root.get<std::string>("sim.<xmlattr>.referenceAreaOverridden", "false") == "true";
   if(overridden)
   {
      rocket.setReferenceArea(root.get<double>("sim.<xmlattr>.referenceArea", rocket.getReferenceArea()));
   }

   // Motor by common name, re-resolved against the supplied database (a miss warns, never throws).
   if(const auto name = root.get_optional<std::string>("motor.<xmlattr>.commonName"))
   {
      if(const auto m = motors.getMotorModel(*name))
      {
         rocket.setMotorModel(*m);
      }
      else
      {
         utils::Logger::getInstance()->warn(
            "DesignSerializer: motor '" + *name + "' not found in the database; loaded without a motor");
      }
   }
}

} // namespace model
