#include "ThrustCurveAPI.h"

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


MotorModel ThrustCurveAPI::getMotorData(const std::string& motorId)
{
   std::stringstream endpoint;
   endpoint << hostname << "download.json?motorId=" << motorId << "&data=samples";
   std::vector<std::string> extraHeaders = {};

   std::string res = curlConnection.get(endpoint.str(), extraHeaders);

   MotorModel mm;
   mm.setDataFromJsonString(res);
   return mm;
}

} // namespace utils
