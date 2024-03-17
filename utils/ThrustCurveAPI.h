#ifndef UTILS_THRUSTCURVEAPI_H
#define UTILS_THRUSTCURVEAPI_H


/// \cond
// C headers
// C++ headers
#include <map>
#include <string>
#include <optional>

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
                    const std::string& vaue);

   std::map<std::string, std::string> criteria;

};

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
    * @brief getThrustCurve will download the thrust data for the given Motor using the motorId
    * @param m MotorModel to populate
    */
   model::MotorModel getMotorData(const std::string& motorId);


   /**
 * @brief getMetaData
 */

   ThrustcurveMetadata getMetadata();

   std::vector<model::MotorModel> searchMotors(const SearchCriteria& c);



private:

   const std::string hostname;
   CurlConnection curlConnection;

   std::optional<ThrustCurve> getThrustCurve(const std::string& id);

   // no extra headers, but CurlConnection library wants them
   const std::vector<std::string> extraHeaders{};
};

} // namespace utils

#endif // UTILS_THRUSTCURVEAPI_H
