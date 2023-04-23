
/// \cond
// C headers
// C++ headers
// 3rd party headers
#include <json/json.h>
/// \endcond

// qtrocket headers
#include "utils/ThrustCurveAPI.h"
#include "utils/Logger.h"

namespace utils
{

ThrustCurveAPI::ThrustCurveAPI()
   : hostname("https://www.thrustcurve.org/api/v1/"),
     curlConnection()
{

}

ThrustCurveAPI::~ThrustCurveAPI()
{

}


model::MotorModel ThrustCurveAPI::getMotorData(const std::string& motorId)
{
   std::stringstream endpoint;
   endpoint << hostname << "download.json?motorId=" << motorId << "&data=samples";
   std::vector<std::string> extraHeaders = {};

   std::string res = curlConnection.get(endpoint.str(), extraHeaders);

   /// TODO: fix this
   model::MotorModel mm;
   return mm;
}

ThrustcurveMetadata ThrustCurveAPI::getMetadata()
{

   std::string endpoint = hostname;
   endpoint += "metadata.json";
   std::string result = curlConnection.get(endpoint, extraHeaders);
   ThrustcurveMetadata ret;

   if(!result.empty())
   {
      try
      {
         Json::Reader reader;
         Json::Value jsonResult;
         reader.parse(result, jsonResult);

         for(Json::ValueConstIterator iter = jsonResult["certOrgs"].begin();
             iter != jsonResult["certOrgs"].end();
             ++iter)
         {
            std::string org = (*iter)["abbrev"].asString();

            if(org == "AMRS")
                ret.certOrgs.emplace_back(model::MotorModel::CERTORG::AMRS);
            else if(org == "CAR")
                ret.certOrgs.emplace_back(model::MotorModel::CERTORG::CAR);
            else if(org == "NAR")
                ret.certOrgs.emplace_back(model::MotorModel::CERTORG::NAR);
            else if(org == "TRA")
                ret.certOrgs.emplace_back(model::MotorModel::CERTORG::TRA);
            else if(org == "UNC")
                ret.certOrgs.emplace_back(model::MotorModel::CERTORG::UNC);
            else
                ret.certOrgs.emplace_back(model::MotorModel::CERTORG::UNK);
         }
         for(Json::ValueConstIterator iter = jsonResult["diameters"].begin();
             iter != jsonResult["diameters"].end();
             ++iter)
         {
            ret.diameters.push_back((*iter).asDouble());
         }
         for(Json::ValueConstIterator iter = jsonResult["impulseClasses"].begin();
             iter != jsonResult["impulseClasses"].end();
             ++iter)
         {
            ret.impulseClasses.emplace_back((*iter).asString());
         }
         for(Json::ValueConstIterator iter = jsonResult["manufacturers"].begin();
             iter != jsonResult["manufacturers"].end();
             ++iter)
         {
            ret.manufacturers[(*iter)["abbrev"].asString()] = (*iter)["name"].asString();
         }
         for(Json::ValueConstIterator iter = jsonResult["types"].begin();
             iter != jsonResult["types"].end();
             ++iter)
         {
            std::string type = (*iter)["types"].asString();
            if(type == "SU")
                ret.types.emplace_back(model::MotorModel::MOTORTYPE::SU);
            else if(type == "reload")
                ret.types.emplace_back(model::MotorModel::MOTORTYPE::RELOAD);
            else
                ret.types.emplace_back(model::MotorModel::MOTORTYPE::HYBRID);
         }
      }
      catch(const std::exception& e)
      {

         std::string err("Unable to parse JSON from Thrustcurve metadata request. Error: ");
         err += e.what();

         Logger::getInstance()->error(err);
      }
   }

   return ret;

}

std::vector<model::MotorModel> ThrustCurveAPI::searchMotors(const SearchCriteria& c)
{
   std::vector<model::MotorModel> retVal;
   std::string endpoint = hostname;
   endpoint += "search.json?";
   for(const auto& i : c.criteria)
   {
      endpoint += i.first;
      endpoint += "=";
      endpoint += i.second;
      endpoint += "&";
   }
   endpoint = endpoint.substr(0, endpoint.length() - 1);


   Logger::getInstance()->debug("endpoint: " + endpoint);
   std::string result = curlConnection.get(endpoint, extraHeaders);
   if(!result.empty())
   {
      try
      {
         Json::Reader reader;
         Json::Value jsonResult;
         reader.parse(result, jsonResult);

         for(Json::ValueConstIterator iter = jsonResult["results"].begin();
             iter != jsonResult["results"].end();
             ++iter)
         {
            model::MotorModel motorModel;
            model::MotorModel::MetaData mm;
            mm.commonName = (*iter)["commonName"].asString();

            std::string availability = (*iter)["availability"].asString();
            if(availability == "regular")
                mm.availability = model::MotorModel::MotorAvailability(model::MotorModel::AVAILABILITY::REGULAR);
            else
                mm.availability = model::MotorModel::MotorAvailability(model::MotorModel::AVAILABILITY::OOP);

            mm.avgThrust = (*iter)["avgThrustN"].asDouble();
            mm.burnTime  = (*iter)["burnTimeS"].asDouble();
            // TODO fill in certOrg
            // TODO fill in delays
            mm.designation = (*iter)["designation"].asString();
            mm.diameter    = (*iter)["diameter"].asDouble();
            mm.impulseClass = (*iter)["impulseClass"].asString();
            mm.length       = (*iter)["length"].asDouble();
            std::string manu = (*iter)["manufacturer"].asString();
            if(manu == "AeroTech")
                mm.manufacturer = model::MotorModel::MOTORMANUFACTURER::AEROTECH;
            //mm.manufacturer = (*iter)["manufacturer"].asString();
            mm.maxThrust    = (*iter)["maxThrustN"].asDouble();
            mm.motorIdTC    = (*iter)["motorId"].asString();
            mm.propType     = (*iter)["propInfo"].asString();
            mm.propWeight   = (*iter)["propWeightG"].asDouble();
            mm.sparky       = (*iter)["sparky"].asBool();
            mm.totalImpulse = (*iter)["totImpulseNs"].asDouble();
            mm.totalWeight  = (*iter)["totalWeightG"].asDouble();

            std::string type = (*iter)["type"].asString();
            if(type == "SU")
                mm.type = model::MotorModel::MotorType(model::MotorModel::MOTORTYPE::SU);
            else if(type == "reload")
                mm.type = model::MotorModel::MotorType(model::MotorModel::MOTORTYPE::RELOAD);
            else
                mm.type = model::MotorModel::MotorType(model::MotorModel::MOTORTYPE::HYBRID);

            motorModel.moveMetaData(std::move(mm));
            retVal.push_back(motorModel);
         }
      }
      catch(const std::exception& e)
      {

         std::string err("Unable to parse JSON from Thrustcurve metadata request. Error: ");
         err += e.what();

         Logger::getInstance()->error(err);
      }
   }
   return retVal;
}

void SearchCriteria::addCriteria(const std::string& name,
                                 const std::string& value)
{
   criteria[name] = value;
}

} // namespace utils
