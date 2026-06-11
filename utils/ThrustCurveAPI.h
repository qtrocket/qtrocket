#ifndef UTILS_THRUSTCURVEAPI_H
#define UTILS_THRUSTCURVEAPI_H


/// \cond
// C headers
// C++ headers
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/CurlConnection.h"
#include "model/MotorModel.h"

namespace utils
{

class ThrustcurveMetadata
{
public:
   ThrustcurveMetadata() = default;
   ~ThrustcurveMetadata() = default;

   ThrustcurveMetadata(const ThrustcurveMetadata&) = default;
   ThrustcurveMetadata(ThrustcurveMetadata&&) = default;

   ThrustcurveMetadata& operator=(const ThrustcurveMetadata&) = default;
   ThrustcurveMetadata& operator=(ThrustcurveMetadata&&) = default;

//private:
   std::vector<model::MotorModel::CertOrg> certOrgs;
   std::vector<double> diameters;
   std::vector<std::string> impulseClasses;
   std::map<std::string, std::string> manufacturers;
   std::vector<model::MotorModel::MotorType> types;

};

class SearchCriteria
{
public:
   SearchCriteria() = default;
   ~SearchCriteria() = default;
   SearchCriteria(const SearchCriteria&) = default;
   SearchCriteria(SearchCriteria&&) = default;

   SearchCriteria& operator=(const SearchCriteria&) = default;
   SearchCriteria& operator=(SearchCriteria&&) = default;

   void addCriteria(const std::string& name,
                    const std::string& value);

   std::map<std::string, std::string> criteria;

};

/**
 * @brief Parsed payload of a search.json response. Carries metadata only;
 *        thrust curves are fetched separately per motor via download.json.
 */
struct SearchResponse
{
   std::vector<model::MotorModel::MetaData> motors;
   int matches{0}; ///< server-reported total; may exceed motors.size()
};

// Pure parsers over raw thrustcurve.org response bodies, split out from the
// HTTP layer so they can be unit-tested with canned JSON. nullopt means the
// body was malformed (logged); a response whose "error" field is set is
// logged and yields empty-but-valid results.
std::optional<SearchResponse> parseSearchResponse(const std::string& json);
std::optional<ThrustcurveMetadata> parseMetadataResponse(const std::string& json);
/// Selects ONE simfile's samples: the first RASP entry with samples, else the
/// first entry of any format with samples. Never concatenates simfiles.
std::optional<std::vector<std::pair<double, double>>>
parseDownloadResponse(const std::string& json);

/**
 * @brief This API for Thrustcurve.org - It will provide an interface for querying thrustcurve.org
 * for motor data
 *
 */
class ThrustCurveAPI
{
public:
   ThrustCurveAPI();
   ~ThrustCurveAPI();

   /**
 * @brief getMetaData
 */

   ThrustcurveMetadata getMetadata();

   std::vector<model::MotorModel> searchMotors(const SearchCriteria& c);



private:

   const std::string hostname;
   CurlConnection curlConnection;

   std::optional<ThrustCurve> getThrustCurve(const std::string& id);
};

} // namespace utils

#endif // UTILS_THRUSTCURVEAPI_H
