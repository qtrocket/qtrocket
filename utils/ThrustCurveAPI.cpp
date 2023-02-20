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

} // namespace utils
