// class header
#include "model/MotorModelDatabase.h"

/// \cond
// C headers
// C++ headers
#include <cmath>
#include <format>
#include <utility>
// 3rd party headers
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
/// \endcond

// qtrocket project headers
#include "utils/Logger.h"
#include "model/RASPLoader.h"
#include "model/RSEDatabaseLoader.h"
#include "model/ThrustCurveClient.h"

namespace model
{

MotorModelDatabase::MotorModelDatabase()
   : MotorModelDatabase(std::unique_ptr<ThrustCurveAPI>{})
{
}

MotorModelDatabase::MotorModelDatabase(std::unique_ptr<ThrustCurveAPI> thrustCurveApi)
   : motorModelMap(),
     tcApi(std::move(thrustCurveApi))
{
}

MotorModelDatabase::~MotorModelDatabase()
{
}

void MotorModelDatabase::addMotorModel(const model::MotorModel& m)
{
   std::string name = m.data.commonName;
   if(motorModelMap.find(name) != motorModelMap.end())
   {
      utils::Logger::getInstance()->debug("Replacing MotorModel " + name + " in MotorModelDatabase");
   }
   else
   {
      utils::Logger::getInstance()->info("Adding MotorModel " + name + " to MotorModelDatabase");
   }
   motorModelMap[name] = m;
}

void MotorModelDatabase::addMotorModels(const std::vector<model::MotorModel>& models)
{
   for(const auto& i : models)
   {
      addMotorModel(i);
   }
}

std::size_t MotorModelDatabase::importRSEFile(const std::string& path)
{
   // RSEDatabaseLoader (owned here, never seen by GUI/CLI) parses the file; copy its motors into our
   // map so selection goes through this database regardless of source.
   const std::size_t before = motorModelMap.size();
   RSEDatabaseLoader loader(path);
   addMotorModels(loader.getMotors());
   // Net new entries, not the raw file count: the map is keyed by common name, so same-name motors
   // collapse onto one entry, and a re-import correctly reports 0.
   return motorModelMap.size() - before;
}

std::size_t MotorModelDatabase::importRASPFile(const std::string& path)
{
   const std::size_t before = motorModelMap.size();
   RASPLoader loader(path);
   addMotorModels(loader.getMotors());
   return motorModelMap.size() - before;
}

std::optional<model::MotorModel> MotorModelDatabase::getMotorModel(const std::string& name)
{
   auto mm = motorModelMap.find(name);
   if(mm == motorModelMap.end())
   {
      utils::Logger::getInstance()->debug("Unable to locate " + name + " in MotorModel database");

      return std::nullopt;

   }
   else
   {
      utils::Logger::getInstance()->debug("Retrieved " + name + " from MotorModel database");
      return motorModelMap[name];
   }
}

std::vector<MotorSummary> MotorModelDatabase::listMotors(const MotorQuery& q) const
{
   auto matches = [&q](const model::MotorModel& m) -> bool
   {
      if(q.manufacturer && m.data.manufacturer.str() != *q.manufacturer)
         return false;
      if(q.impulseClass && m.data.impulseClass != *q.impulseClass)
         return false;
      // Motor diameters are nominal integer millimetres at least 1 mm apart, so a 0.5 mm
      // tolerance distinguishes classes while tolerating doubles round-tripped through XML.
      if(q.diameter && std::abs(m.data.diameter - *q.diameter) > 0.5)
         return false;
      if(q.nameContains && m.data.commonName.find(*q.nameContains) == std::string::npos)
         return false;
      return true;
   };

   std::vector<MotorSummary> result;
   // motorModelMap is keyed by commonName, so iteration is already sorted by name.
   for(const auto& entry : motorModelMap)
   {
      const model::MotorModel& m = entry.second;
      if(matches(m))
         result.push_back(toSummary(m));
   }
   return result;
}

MotorSummary MotorModelDatabase::toSummary(const model::MotorModel& m)
{
   return MotorSummary{
      .commonName   = m.data.commonName,
      .manufacturer = m.data.manufacturer.str(),
      .avgThrust    = m.data.avgThrust,
      .totalImpulse = m.data.totalImpulse,
      .diameter     = m.data.diameter,
      .impulseClass = m.data.impulseClass};
}

ThrustCurveAPI& MotorModelDatabase::thrustCurveApi()
{
   if(!tcApi)
      tcApi = std::make_unique<ThrustCurveClient>();
   return *tcApi;
}

MotorSearchFacets MotorModelDatabase::getOnlineSearchFacets()
{
   ThrustcurveMetadata meta = thrustCurveApi().getMetadata();

   MotorSearchFacets facets;
   facets.diameters = meta.diameters;
   facets.impulseClasses = meta.impulseClasses;
   // meta.manufacturers maps code -> full name; the search API keys on the code.
   for(const auto& [code, name] : meta.manufacturers)
      facets.manufacturers.push_back(code);
   return facets;
}

std::vector<MotorSummary> MotorModelDatabase::searchOnline(const MotorQuery& q)
{
   SearchCriteria criteria;
   if(q.manufacturer)
      criteria.addCriteria("manufacturer", *q.manufacturer);
   if(q.impulseClass)
      criteria.addCriteria("impulseClass", *q.impulseClass);
   if(q.diameter)
      // std::format prints 38.0 as "38" and 13.5 as "13.5" -- the form the API expects.
      criteria.addCriteria("diameter", std::format("{}", *q.diameter));

   std::vector<MotorSummary> found;
   for(const auto& motor : thrustCurveApi().searchMotors(criteria))
   {
      addMotorModel(motor); // merge into the database so getMotorModel()/listMotors() see it
      found.push_back(toSummary(motor));
   }
   return found;
}

void MotorModelDatabase::saveMotorDatabase(const std::string& filename)
{

   namespace pt = boost::property_tree;

   // top-level tree
   pt::ptree tree;
   tree.put("QtRocketMotorDatabase.<xmlattr>.version", "0.1");
   for(const auto& i : motorModelMap)
   {
      pt::ptree motor;
      const auto& m = i.second;
      motor.put("<xmlattr>.name", m.data.commonName);
      motor.put("availability", m.data.availability.str());
      motor.put("avgThrust", m.data.avgThrust);
      motor.put("burnTime", m.data.burnTime);
      motor.put("certOrg", m.data.certOrg.str());
      motor.put("commonName", m.data.commonName);
      motor.put("designation", m.data.designation);
      motor.put("diameter", m.data.diameter);
      motor.put("impulseClass", m.data.impulseClass);
      motor.put("infoUrl", m.data.infoUrl);
      motor.put("length", m.data.length);
      motor.put("manufacturer", m.data.manufacturer.str());
      motor.put("maxThrust", m.data.maxThrust);
      motor.put("motorIdTC", m.data.motorIdTC);
      motor.put("propType", m.data.propType);
      motor.put("propWeight", m.data.propWeight);
      motor.put("sparky", m.data.sparky ? "true" : "false");
      motor.put("totalImpulse", m.data.totalImpulse);
      motor.put("totalWeight", m.data.totalWeight);
      motor.put("type", m.data.type.str());
      motor.put("lastUpdated", m.data.lastUpdated);

      // delays tag is a csv string. Guard against an empty delays vector (thrustcurve.org search
      // results carry none): the old size()-1 form underflowed and indexed out of bounds.
      std::stringstream delays;
      for (std::size_t j = 0; j < m.data.delays.size(); ++j)
      {
          if(j > 0)
              delays << ",";
          delays << std::to_string(m.data.delays[j]);
      }
      motor.put("delays", delays.str());

      // thrust data
      {
         pt::ptree tc;
         std::vector<std::pair<double, double>> thrust = m.getThrustCurve().getThrustCurveData();
         for(const auto& k : thrust)
         {
            pt::ptree thrustNode;
            thrustNode.put("<xmlattr>.time", k.first);
            thrustNode.put("<xmlattr>.force", k.second);
            tc.add_child("thrust", thrustNode);
         }
         motor.add_child("thrustCurve", tc);
      }
      tree.add_child("QtRocketMotorDatabase.MotorModels.motor", motor);
   }
   pt::xml_writer_settings<std::string> settings(' ', 2);
   pt::write_xml(filename, tree, std::locale(), settings);
}

void MotorModelDatabase::loadMotorDatabase(const std::string& filename)
{
   namespace pt = boost::property_tree;
   namespace mm = model;

   pt::ptree tree;
   pt::read_xml(filename, tree);

   for(const auto& [key, motor] : tree.get_child("QtRocketMotorDatabase.MotorModels"))
   {
      if(key != "motor")
         continue; // skip attributes / any non-motor nodes

      mm::MotorModel::MetaData md;
      md.availability  = mm::MotorModel::MotorAvailability(
                            mm::MotorModel::MotorAvailability::toEnum(motor.get<std::string>("availability", "regular")));
      md.avgThrust     = motor.get<double>("avgThrust", 0.0);
      md.burnTime      = motor.get<double>("burnTime", 0.0);
      md.certOrg       = mm::MotorModel::CertOrg(
                            mm::MotorModel::CertOrg::toEnum(motor.get<std::string>("certOrg", "Uncertified")));
      md.commonName    = motor.get<std::string>("commonName", "");
      md.designation   = motor.get<std::string>("designation", "");
      md.diameter      = motor.get<double>("diameter", 0.0);
      md.impulseClass  = motor.get<std::string>("impulseClass", "");
      md.infoUrl       = motor.get<std::string>("infoUrl", "");
      md.length        = motor.get<double>("length", 0.0);
      md.manufacturer  = mm::MotorModel::MotorManufacturer(
                            mm::MotorModel::MotorManufacturer::toEnum(motor.get<std::string>("manufacturer", "Unknown")));
      md.maxThrust     = motor.get<double>("maxThrust", 0.0);
      md.motorIdTC     = motor.get<std::string>("motorIdTC", "");
      md.propType      = motor.get<std::string>("propType", "");
      md.propWeight    = motor.get<double>("propWeight", 0.0);
      md.sparky        = motor.get<std::string>("sparky", "false") == "true";
      md.totalImpulse  = motor.get<double>("totalImpulse", 0.0);
      md.totalWeight   = motor.get<double>("totalWeight", 0.0);
      md.type          = mm::MotorModel::MotorType(
                            mm::MotorModel::MotorType::toEnum(motor.get<std::string>("type", "Single Use")));
      md.lastUpdated   = motor.get<std::string>("lastUpdated", "");

      // delays were written as a comma-separated string
      std::stringstream delays(motor.get<std::string>("delays", ""));
      std::string tok;
      while(std::getline(delays, tok, ','))
      {
         if(!tok.empty())
            md.delays.push_back(std::stoi(tok));
      }

      // thrust curve: <thrustCurve><thrust time=".." force=".."/>...</thrustCurve>
      std::vector<std::pair<double, double>> thrustData;
      if(auto thrustCurve = motor.get_child_optional("thrustCurve"))
      {
         for(const auto& [tkey, tnode] : *thrustCurve)
         {
            if(tkey != "thrust")
               continue;
            thrustData.emplace_back(tnode.get<double>("<xmlattr>.time", 0.0),
                                    tnode.get<double>("<xmlattr>.force", 0.0));
         }
      }

      mm::MotorModel motorModel;
      // Order matters: setMetaData() triggers computeMassCurve(), which integrates the thrust
      // curve, so the curve must be in place first (mirrors RSEDatabaseLoader).
      motorModel.addThrustCurve(ThrustCurve(thrustData));
      motorModel.setMetaData(md);
      addMotorModel(motorModel);
   }
}

} // namespace model
