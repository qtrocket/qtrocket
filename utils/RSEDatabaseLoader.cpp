
/// \cond
// C headers
// C++ headers
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstring>

// 3rd party headers
#include <boost/property_tree/xml_parser.hpp>
/// \endcond

// qtrocket headers
#include "utils/RSEDatabaseLoader.h"
#include "QtRocket.h"
#include "Logger.h"

namespace utils {

RSEDatabaseLoader::RSEDatabaseLoader(const std::string& filename)
   : motors(),
     tree()
{
   boost::property_tree::read_xml(filename, tree);

   // Get the first engine
   for(boost::property_tree::ptree::value_type& v : tree.get_child("engine-database.engine-list"))
   {
      buildAndAppendMotorModel(v.second);
   }

   QtRocket::getInstance()->addMotorModels(motors);
}

RSEDatabaseLoader::~RSEDatabaseLoader()
{}

model::MotorModel RSEDatabaseLoader::getMotorModelByName(const std::string &name)
{

   auto mm = std::find_if(motors.begin(), motors.end(),
                                        [&name](const auto& i) { return name == i.data.commonName; });
   if(mm == motors.end())
   {
      Logger::getInstance()->error("Unable to locate " + name + " in RSE database");
      return model::MotorModel();
   }
   return *mm;
}

void RSEDatabaseLoader::buildAndAppendMotorModel(boost::property_tree::ptree& v)
{
   model::MotorModel::MetaData mm;
   mm.availability = model::MotorModel::MotorAvailability(model::MotorModel::AVAILABILITY::REGULAR);
   mm.avgThrust = v.get<double>("<xmlattr>.avgThrust", 0.0);
   mm.burnTime  = v.get<double>("<xmlattr>.burn-time", 0.0);
   mm.certOrg   = model::MotorModel::CertOrg(model::MotorModel::CERTORG::UNK);
   mm.commonName = v.get<std::string>("<xmlattr>.code", "");

   // mm.delays = extract vector from csv list
   std::string delays = v.get<std::string>("<xmlattr>.delays", "1000");
   std::size_t pos{0};
   std::string tok;
   while ((pos = delays.find(",")) != std::string::npos)
   {
       tok = delays.substr(0, pos);
       mm.delays.push_back(std::atoi(tok.c_str()));
       delays.erase(0, pos + 1);
   }
   mm.delays.push_back(std::atoi(delays.c_str()));

   // mm.designation = What is this?

   mm.diameter = v.get<double>("<xmlattr>.dia", 0.0);
   // impulse class is the motor letter designation. extract from the first character
   // of the commonName since it isn't given explicity in the RSE file
   mm.impulseClass = mm.commonName[0];

   // infoUrl not present in RSE file
   mm.infoUrl = "";
   mm.length = v.get<double>("<xmlattr>.len", 0.0);
   mm.manufacturer = model::MotorModel::MotorManufacturer::toEnum(v.get<std::string>("<xmlattr>.mfg", ""));
   mm.maxThrust = v.get<double>("<xmlattr>.peakThrust", 0.0);
   mm.totalWeight = v.get<double>("<xmlattr>.initWt", 0.0) / 1000.0; // convert g -> kg
   mm.propWeight = v.get<double>("<xmlattr>.propWt", 0.0) / 1000.0;  // convert g -> kg
   mm.totalImpulse = v.get<double>("<xmlattr>.Itot", 0.0);

   mm.type = model::MotorModel::MotorType::toEnum(v.get<std::string>("<xmlattr>.Type"));

   // Now get the thrust data
   std::vector<std::pair<double, double>> thrustData;
   for(boost::property_tree::ptree::value_type& w : v.get_child("data"))
   {
      double tdata = w.second.get<double>("<xmlattr>.t");
      double fdata = w.second.get<double>("<xmlattr>.f");
      thrustData.push_back(std::make_pair(tdata, fdata));
   }

   ThrustCurve tc(thrustData);
   model::MotorModel motorModel;
   motorModel.addThrustCurve(tc);
   motorModel.moveMetaData(std::move(mm));
   motors.emplace_back(std::move(motorModel));
}


} // namespace utils
