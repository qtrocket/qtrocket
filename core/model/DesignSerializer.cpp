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
#include "model/PartsModel.h"  // PartNode -- the save walk reads the node tree
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

pt::ptree writeLink(const part::StationLink& link)
{
   pt::ptree n;
   n.put("<xmlattr>.seat", part::seatKindToString(link.seat));
   n.put("<xmlattr>.parentStation", link.parentStation01);
   n.put("<xmlattr>.childStation", link.childStation01);
   n.put("<xmlattr>.gap", link.gap);
   return n;
}

// True when a link carries no authored intent beyond the zero-config default (child fore plane abuts
// parent aft plane, no gap). Such a link is elided on write and recovered as the default on read.
// childRot is not persisted (6-DOF, identity in 3-DOF), so only the scalar fields and seat compare.
bool isDefaultLink(const part::StationLink& link)
{
    const part::StationLink d{};
    return link.seat == d.seat && link.parentStation01 == d.parentStation01
         && link.childStation01 == d.childStation01 && link.gap == d.gap;
}

// Recursively serialize a (non-Motor) node and its non-Motor descendants. The node's own link is
// its placement intent relative to its parent (a default link on the root, ignored on load).
// Absolute pose is never serialized -- the resolver re-derives it on load.
pt::ptree writePart(const PartNode& node)
{
    pt::ptree pn;
    pn.put("<xmlattr>.type", node.part().typeName());
    pn.put("<xmlattr>.name", node.part().getName());
    pn.add_child("params", writeParams(part::params(node.part())));
    // Emit <link> only for non-default intent; a default-equal link (zero-config abut) is elided and
    // recovered as the default on read.
    if(!isDefaultLink(node.link()))
    {
        pn.add_child("link", writeLink(node.link()));
    }

    pt::ptree children;
    for(const auto& child : node.children())
    {
        // The motor is serialized separately, by common name (see save) -- not as a tree part, since it
        // cannot be rebuilt by the geometry factory.
        if(dynamic_cast<const part::Motor*>(&child->part()) != nullptr) { continue; }
        children.add_child("part", writePart(*child)); // add_child (NOT put) for siblings
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

// Parse the <link> child of @p owner into a StationLink (fail-closed on an unknown seat name).
part::StationLink readLink(const pt::ptree& owner)
{
    const std::string seatStr = owner.get<std::string>("link.<xmlattr>.seat", "Abut");
    const auto        seat    = part::seatKindFromString(seatStr);
    if(!seat) // fail-closed, on the same path as makePart's unknown-type rejection
    {
        throw std::runtime_error("DesignSerializer: unknown seat kind '" + seatStr + "'");
    }
    return part::StationLink{
        owner.get<double>("link.<xmlattr>.parentStation", 0.0),
        owner.get<double>("link.<xmlattr>.childStation", 1.0),
        owner.get<double>("link.<xmlattr>.gap", 0.0),
        *seat,
        Quaternion::Identity()};
}

std::unique_ptr<PartNode> buildNode(const pt::ptree& partNode)
{
    const std::string type = partNode.get<std::string>("<xmlattr>.type");
    if(type == "Motor")
    {
        // writePart never emits a Motor as a tree part (it skips them); motors round-trip by common
        // name via <motor>. A Motor here means a hand-edited or foreign file -- reject it clearly.
        throw std::runtime_error(
            "DesignSerializer: a Motor cannot be a tree part; it loads by common name via <motor>");
    }
    std::unique_ptr<PartNode> node =
        PartNode::make(part::makePart(type, readParams(partNode))); // throws on bad/unknown

    if(const auto kids = partNode.get_child_optional("children"))
    {
        for(const auto& [key, childNode] : *kids)
        {
            if(key != "part") { continue; } // skip any non-<part> entry (e.g. an <xmlattr> pseudo-node)
            std::unique_ptr<PartNode> child = buildNode(childNode);

            // 0.2 placement: a <link> (explicit intent) or neither element (an elided default-equal
            // link, attaches by the zero-config abut default). A legacy 0.1 <offset> (CM-to-CM) is
            // rejected rather than silently mis-placed.
            part::StationLink link{};
            if(childNode.get_child_optional("link"))
            {
                link = readLink(childNode);
            }
            else if(childNode.get_child_optional("offset"))
            {
                throw std::runtime_error(
                    "DesignSerializer: legacy 0.1 <offset> placement is no longer supported; this file "
                    "predates the 0.2 <link> format and must be re-created");
            }
            node->addChild(std::move(child), link);
        }
    }
    return node;
}
} // anonymous namespace

void DesignSerializer::save(const RocketModel& rocket, const std::string& filename)
{
    pt::ptree tree;
    // Format version "major.minor": load accepts any minor within major 0, rejects other majors. 0.2
    // uses the <link> placement element (0.1 wrote a CM-to-CM <offset>, no longer supported).
    tree.put("QtRocketDesign.<xmlattr>.version", "0.2");
    tree.put("QtRocketDesign.design.<xmlattr>.name", rocket.getName());

    tree.add_child("QtRocketDesign.part", writePart(*rocket.parts().root()));

    if(rocket.isMotorSet())
    {
        tree.put("QtRocketDesign.motor.<xmlattr>.commonName", rocket.getMotorModel().data.commonName);
        // The motor is an ordinary node; persist its seat like any other edge (elided if default).
        if(const PartNode* mn = rocket.parts().motorNode(); mn != nullptr && !isDefaultLink(mn->link()))
        {
            tree.add_child("QtRocketDesign.motor.link", writeLink(mn->link()));
        }
    }

    // The <sim> block carries only the RocketModel-owned aero options. Launch/environment options
    // (velocity, angle, atmosphere, gravity, integrator) are not RocketModel state and are not
    // serialized here -- the CLI manages them as session config.
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

    // Build the whole tree (detached) before touching the rocket, then install in one shot so a
    // malformed file leaves the existing rocket untouched.
    std::unique_ptr<PartNode> newRoot = buildNode(root.get_child("part"));

    // Motor by common name, re-resolved against the supplied database (a miss warns, never throws).
    // An ordinary node: attached under the root with its persisted link (default abut when elided).
    if(const auto name = root.get_optional<std::string>("motor.<xmlattr>.commonName"))
    {
        if(const auto m = motors.getMotorModel(*name))
        {
            part::StationLink motorLink{};
            if(root.get_child_optional("motor.link"))
            {
                motorLink = readLink(root.get_child("motor"));
            }
            newRoot->addChild(PartNode::make(std::make_unique<part::Motor>("Motor", *m)), motorLink);
        }
        else
        {
            utils::Logger::getInstance()->warn(
                "DesignSerializer: motor '" + *name + "' not found in the database; loaded without a motor");
        }
    }

    rocket.installDesign(std::move(newRoot));
    rocket.setName(root.get<std::string>("design.<xmlattr>.name", ""));

    // Apply the sim/aero block after the install so a restored manual override wins over its reset.
    rocket.setDragCoefficient(root.get<double>("sim.<xmlattr>.dragCoefficient", rocket.getDragCoefficient()));
    const bool overridden =
        root.get<std::string>("sim.<xmlattr>.referenceAreaOverridden", "false") == "true";
    if(overridden)
    {
        rocket.setReferenceArea(root.get<double>("sim.<xmlattr>.referenceArea", rocket.getReferenceArea()));
    }
}

} // namespace model
