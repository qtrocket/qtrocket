
/// \cond
// C headers
// C++ headers
#include <memory>
#include <optional>

// 3rd party headers
#include <json/json.h>
/// \endcond

// qtrocket headers
#include "model/ThrustCurveClient.h"
#include "utils/Logger.h"

namespace
{

/// Parse a raw response body; false (with a logged error) on malformed JSON.
bool parseJson(const std::string& text, Json::Value& root, const char* context)
{
   Json::CharReaderBuilder builder;
   std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
   std::string errs;
   if(!reader->parse(text.data(), text.data() + text.size(), &root, &errs))
   {
      utils::Logger::getInstance()->error(
         std::string("Unable to parse JSON from thrustcurve.org ") + context +
         " response. Error: " + errs);
      return false;
   }
   return true;
}

/// The API reports request-level problems in an "error" field of an otherwise
/// well-formed response; surface it in the log.
void logApiError(const Json::Value& root, const char* context)
{
   if(root.isObject() && root.isMember("error"))
   {
      const std::string err = root["error"].asString();
      if(!err.empty())
      {
         utils::Logger::getInstance()->warn(
            std::string("thrustcurve.org ") + context + " response reported: " + err);
      }
   }
}

} // namespace

namespace model
{

std::optional<std::vector<std::pair<double, double>>>
parseDownloadResponse(const std::string& json)
{
   Json::Value root;
   if(!parseJson(json, root, "motor data"))
   {
      return std::nullopt;
   }
   logApiError(root, "motor data");

   try
   {
      // A motor usually has several data files (simfiles) in arbitrary order;
      // their sample sets are alternative descriptions of the same burn, so we
      // must pick exactly one: the first RASP entry if any, else the first
      // entry of any format. Concatenating them would interleave time series.
      std::vector<std::pair<double, double>> fallback;
      for(const auto& simfile : root["results"])
      {
         std::vector<std::pair<double, double>> samples;
         for(const auto& sample : simfile["samples"])
         {
            samples.emplace_back(sample["time"].asDouble(),
                                 sample["thrust"].asDouble());
         }
         if(samples.empty())
         {
            continue;
         }
         if(simfile["format"].asString() == "RASP")
         {
            return samples;
         }
         if(fallback.empty())
         {
            fallback = std::move(samples);
         }
      }
      return fallback;
   }
   catch(const std::exception& e)
   {
      utils::Logger::getInstance()->error(
         std::string("Unexpected JSON in thrustcurve.org motor data response. Error: ") +
         e.what());
      return std::nullopt;
   }
}

std::optional<ThrustcurveMetadata> parseMetadataResponse(const std::string& json)
{
   Json::Value root;
   if(!parseJson(json, root, "metadata"))
   {
      return std::nullopt;
   }
   logApiError(root, "metadata");

   try
   {
      ThrustcurveMetadata ret;
      for(const auto& certOrg : root["certOrgs"])
      {
         std::string org = certOrg["abbrev"].asString();

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
      for(const auto& diameter : root["diameters"])
      {
         ret.diameters.push_back(diameter.asDouble());
      }
      for(const auto& impulseClass : root["impulseClasses"])
      {
         ret.impulseClasses.emplace_back(impulseClass.asString());
      }
      for(const auto& manufacturer : root["manufacturers"])
      {
         ret.manufacturers[manufacturer["abbrev"].asString()] =
            manufacturer["name"].asString();
      }
      // "types" elements are plain strings ("SU", "reload", "hybrid"),
      // unlike certOrgs/manufacturers which are {name, abbrev} objects.
      for(const auto& type : root["types"])
      {
         std::string typeStr = type.asString();
         if(typeStr == "SU")
            ret.types.emplace_back(model::MotorModel::MOTORTYPE::SU);
         else if(typeStr == "reload")
            ret.types.emplace_back(model::MotorModel::MOTORTYPE::RELOAD);
         else
            ret.types.emplace_back(model::MotorModel::MOTORTYPE::HYBRID);
      }
      return ret;
   }
   catch(const std::exception& e)
   {
      utils::Logger::getInstance()->error(
         std::string("Unexpected JSON in thrustcurve.org metadata response. Error: ") +
         e.what());
      return std::nullopt;
   }
}

std::optional<SearchResponse> parseSearchResponse(const std::string& json)
{
   Json::Value root;
   if(!parseJson(json, root, "search"))
   {
      return std::nullopt;
   }
   logApiError(root, "search");

   try
   {
      SearchResponse response;
      response.matches = root["matches"].asInt();
      for(const auto& result : root["results"])
      {
         model::MotorModel::MetaData mm;
         mm.commonName = result["commonName"].asString();

         std::string availability = result["availability"].asString();
         if(availability == "regular")
            mm.availability = model::MotorModel::MotorAvailability(model::MotorModel::AVAILABILITY::REGULAR);
         else if(availability == "occasional")
            mm.availability = model::MotorModel::MotorAvailability(model::MotorModel::AVAILABILITY::OCCASIONAL);
         else
            mm.availability = model::MotorModel::MotorAvailability(model::MotorModel::AVAILABILITY::OOP);

         mm.avgThrust = result["avgThrustN"].asDouble();
         mm.burnTime  = result["burnTimeS"].asDouble();
         // TODO fill in certOrg
         // TODO fill in delays
         mm.designation = result["designation"].asString();
         mm.diameter    = result["diameter"].asDouble();
         mm.impulseClass = result["impulseClass"].asString();
         mm.length       = result["length"].asDouble();
         mm.manufacturer = model::MotorModel::MotorManufacturer::toEnum(
                              result["manufacturer"].asString());
         mm.maxThrust    = result["maxThrustN"].asDouble();
         mm.motorIdTC    = result["motorId"].asString();
         mm.propType     = result["propInfo"].asString();
         mm.propWeight   = result["propWeightG"].asDouble();
         mm.sparky       = result["sparky"].asBool();
         mm.totalImpulse = result["totImpulseNs"].asDouble();
         mm.totalWeight  = result["totalWeightG"].asDouble();

         std::string type = result["type"].asString();
         if(type == "SU")
            mm.type = model::MotorModel::MotorType(model::MotorModel::MOTORTYPE::SU);
         else if(type == "reload")
            mm.type = model::MotorModel::MotorType(model::MotorModel::MOTORTYPE::RELOAD);
         else
            mm.type = model::MotorModel::MotorType(model::MotorModel::MOTORTYPE::HYBRID);

         response.motors.push_back(std::move(mm));
      }
      return response;
   }
   catch(const std::exception& e)
   {
      utils::Logger::getInstance()->error(
         std::string("Unexpected JSON in thrustcurve.org search response. Error: ") +
         e.what());
      return std::nullopt;
   }
}

ThrustCurveClient::ThrustCurveClient()
   : hostname("https://www.thrustcurve.org/"),
     curlConnection()
{

}

ThrustCurveClient::~ThrustCurveClient()
{

}

std::optional<ThrustCurve> ThrustCurveClient::getThrustCurve(const std::string& id)
{
   std::string endpoint = hostname + "api/v1/download.json?motorId=" + id + "&data=samples";

   std::string res = curlConnection.get(endpoint);
   if(res.empty())
   {
      return std::nullopt;
   }

   std::optional<std::vector<std::pair<double, double>>> samples =
      parseDownloadResponse(res);
   // No samples (e.g. a motor with no data files) is not a usable curve.
   if(!samples || samples->empty())
   {
      return std::nullopt;
   }
   return ThrustCurve(*samples);
}

ThrustcurveMetadata ThrustCurveClient::getMetadata()
{
   std::string endpoint = hostname + "api/v1/metadata.json";

   std::string result = curlConnection.get(endpoint);
   if(result.empty())
   {
      return ThrustcurveMetadata();
   }

   std::optional<ThrustcurveMetadata> metadata = parseMetadataResponse(result);
   return metadata ? *metadata : ThrustcurveMetadata();
}

std::vector<model::MotorModel> ThrustCurveClient::searchMotors(const SearchCriteria& c)
{
   std::vector<model::MotorModel> retVal;
   std::string endpoint = hostname;
   endpoint += "api/v1/search.json?";
   for(const auto& [name, value] : c.criteria)
   {
      endpoint += name;
      endpoint += "=";
      endpoint += value;
      endpoint += "&";
   }
   // The server returns at most 20 motors unless told otherwise. Cap rather
   // than fetch everything: each result costs one download.json round trip
   // below, performed synchronously on the caller's (often GUI) thread.
   if(c.criteria.contains("maxResults"))
      endpoint.pop_back(); // drop the trailing '&'
   else
      endpoint += "maxResults=100";

   utils::Logger::getInstance()->debug("endpoint: " + endpoint);
   std::string result = curlConnection.get(endpoint);
   if(result.empty())
   {
      return retVal;
   }

   std::optional<SearchResponse> response = parseSearchResponse(result);
   if(!response)
   {
      return retVal;
   }
   if(response->matches > static_cast<int>(response->motors.size()))
   {
      utils::Logger::getInstance()->warn(
         "thrustcurve.org search matched " + std::to_string(response->matches) +
         " motors but only " + std::to_string(response->motors.size()) +
         " were returned; refine the search to see the rest");
   }

   for(model::MotorModel::MetaData& mm : response->motors)
   {
      model::MotorModel motorModel;
      auto tc = getThrustCurve(mm.motorIdTC);
      if(tc)
      {
         motorModel.addThrustCurve(*tc);
      }
      // Add the thrust curve before the metadata: moveMetaData() triggers computeMassCurve(),
      // which integrates the thrust curve, so the curve must be in place first.
      motorModel.moveMetaData(std::move(mm));
      retVal.push_back(motorModel);
   }
   return retVal;
}

void SearchCriteria::addCriteria(const std::string& name,
                                 const std::string& value)
{
   criteria[name] = value;
}

} // namespace model
