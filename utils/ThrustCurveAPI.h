#ifndef UTILS_THRUSTCURVEAPI_H
#define UTILS_THRUSTCURVEAPI_H


#include <string>

#include "CurlConnection.h"

namespace utils
{

/**
 * @brief This is a bit more than just an API for Thrustcurve.org, so the name is unfortunate
 *        It is also an internal database of motor data that it grabs from Thrustcurve.org
 *
 */
class ThrustCurveAPI
{
public:
   ThrustCurveAPI();
   ~ThrustCurveAPI();



private:

   const std::string hostname;
   CurlConnection curlConnection;
};

} // namespace utils

#endif // UTILS_THRUSTCURVEAPI_H
