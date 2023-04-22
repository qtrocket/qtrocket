
/// \cond
// C headers
// C++ headers
#include <iostream>
#include <cmath>

// 3rd party headers
#include <boost/property_tree/xml_parser.hpp>
/// \endcond

// qtrocket headers
#include "utils/RSEDatabaseLoader.h"
#include "QtRocket.h"

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

void RSEDatabaseLoader::buildAndAppendMotorModel(boost::property_tree::ptree& v)
{
   MotorModel mm;
   mm.availability = MotorModel::MotorAvailability(MotorModel::AVAILABILITY::REGULAR);
   mm.avgThrust = v.get<double>("<xmlattr>.avgThrust", 0.0);
   mm.burnTime  = v.get<double>("<xmlattr>.burn-time", 0.0);
   mm.certOrg   = MotorModel::CertOrg(MotorModel::CERTORG::UNK);
   mm.commonName = v.get<std::string>("<xmlattr>.code", "");

   // mm.delays = extract vector from csv list

   // mm.designation = What is this?

   // This is in the form of dia="18.", or dia="38."
   {
      std::string dia = v.get<std::string>("<xmlattr>.dia", "");
      dia = dia.substr(0, dia.length() - 1);
      mm.diameter = std::stoi(dia);
   }
   // impulse class is the motor letter designation. extract from the first character
   // of the commonName since it isn't given explicity in the RSE file
   mm.impulseClass = mm.commonName[0];

   // infoUrl not present in RSE file
   mm.infoUrl = "";
   mm.length = v.get<double>("<xmlattr>.len", 0.0);
   mm.manufacturer = v.get<std::string>("<xmlattr>.mfg", "");
   mm.maxThrust = v.get<double>("<xmlattr>.peakThrust", 0.0);
   mm.propWeight = v.get<double>("<xmlattr>.propWt", 0.0);
   mm.totalImpulse = v.get<double>("<xmlattr>.Itot", 0.0);

   {
      std::string type = v.get<std::string>("<xmlattr>.Type");
      MotorModel::MotorType mt(MotorModel::MOTORTYPE::SU);
      if(type.compare("reloadable") == 0)
      {
         mt = MotorModel::MOTORTYPE::RELOAD;
      }
      else if(type.compare("hybrid") == 0)
      {
         mt = MotorModel::MOTORTYPE::HYBRID;
      }
      else
      {
         // single use, which is default
      }
      mm.type = mt;
   }

   // Now get the thrust data
   std::vector<std::pair<double, double>> thrustData;
   for(boost::property_tree::ptree::value_type& w : v.get_child("data"))
   {
      double tdata = w.second.get<double>("<xmlattr>.t");
      double fdata = w.second.get<double>("<xmlattr>.f");
      thrustData.push_back(std::make_pair(tdata, fdata));
   }
   /*
   std::cout << "\n--------------------------------------------\n";
   std::cout << "name: " << mm.commonName << std::endl;
   std::cout << "impulseClass: " << mm.impulseClass << std::endl;
   std::cout << "length: " << mm.length << std::endl;
   std::cout << "manufacturer: " << mm.manufacturer << std::endl;
   std::cout << "maxThrust: " << mm.maxThrust << std::endl;
   std::cout << "propWeight: " << mm.propWeight << std::endl;
   std::cout << "totalImpulse: " << mm.totalImpulse << std::endl;
   std::cout << "--------------------------------------------\n";

   std::cout << "thrust data:\n";
   for(const auto& i : thrustData)
   {
      std::cout << "(" << i.first << ", " << i.second << ")\n";
   }
   */

   motors.emplace_back(std::move(mm));
}


} // namespace utils
