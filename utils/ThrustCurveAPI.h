#ifndef UTILS_THRUSTCURVEAPI_H
#define UTILS_THRUSTCURVEAPI_H


#include <string>

#include "CurlConnection.h"
#include "model/MotorModel.h"
#include "model/Thrustcurve.h"

namespace utils
{

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
   MotorModel getMotorData(const std::string& motorId);




private:

   const std::string hostname;
   CurlConnection curlConnection;
};

} // namespace utils

#endif // UTILS_THRUSTCURVEAPI_H
