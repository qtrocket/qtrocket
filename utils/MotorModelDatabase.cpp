// class header
#include "utils/MotorModelDatabase.h"

/// \cond
// C headers
// C++ headers
// 3rd party headers
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
/// \endcond

// qtrocket project headers
#include "QtRocket.h"

namespace utils
{

MotorModelDatabase::MotorModelDatabase()
   : motorModelMap()
{
}

MotorModelDatabase::~MotorModelDatabase()
{
}

void MotorModelDatabase::addMotorModel(const model::MotorModel& m)
{
   utils::Logger* logger = QtRocket::getInstance()->getLogger();
   std::string name = m.data.commonName;
   if(motorModelMap.find(name) != motorModelMap.end())
   {
      logger->debug("Replacing MotorModel " + name + " in MotorModelDatabase");
   }
   else
   {
      logger->info("Adding MotorModel " + name + " to MotorModelDatabase");
   }
   motorModelMap[name] = m;
}

void MotorModelDatabase::addMotorModels(const std::vector<model::MotorModel>& models)
{
   utils::Logger* logger = QtRocket::getInstance()->getLogger();
   for(const auto& i : models)
   {
      addMotorModel(i);
   }
}

std::optional<model::MotorModel> MotorModelDatabase::getMotorModel(const std::string& name)
{
   auto mm = motorModelMap.find(name);
   if(mm == motorModelMap.end())
   {
      Logger::getInstance()->debug("Unable to locate " + name + " in MotorModel database");

      return std::nullopt;

   }
   else
   {
      Logger::getInstance()->debug("Retrieved " + name + " from MotorModel database");
      return motorModelMap[name];
   }
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
      motor.put("sparky", m.data.sparky ? "true" : "false");
      motor.put("totalImpulse", m.data.totalImpulse);
      motor.put("totalWeight", m.data.totalWeight);
      motor.put("type", m.data.type.str());
      motor.put("lastUpdated", m.data.lastUpdated);

      // delays tag is in the form of a csv string
      std::stringstream delays;
      for (std::size_t i = 0; i < m.data.delays.size() - 1; ++i)
      {
          delays << std::to_string(m.data.delays[i]) << ",";
      }
      delays << std::to_string(m.data.delays[m.data.delays.size() - 1]);
      motor.put("delays", delays.str());

      // thrust data
      {
         pt::ptree tc;
         std::vector<std::pair<double, double>> thrust = m.getThrustCurve().getThrustCurveData();
         for(const auto& j : thrust)
         {
            pt::ptree thrustNode;
            thrustNode.put("<xmlattr>.time", j.first);
            thrustNode.put("<xmlattr>.force", j.second);
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

}

} // namespace utils
