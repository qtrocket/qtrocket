#include "USStandardAtmosphere.h"

#include "utils/math/Constants.h"
#include <cmath>

using namespace utils::math;

namespace sim
{

USStandardAtmosphere::USStandardAtmosphere()
{

}

USStandardAtmosphere::~USStandardAtmosphere()
{

}

double USStandardAtmosphere::getDensity(double altitude)
{
   return Constants::standardDensity * std::exp((-Constants::g0 * Constants::airMolarMass * altitude) / (Constants::Rstar * Constants::standardTemperature));
}

double USStandardAtmosphere::getTemperature(double altitude)
{
   return 0.0;
}
double USStandardAtmosphere::getPressure(double altitude)
{
   return 0.0;
}
} // namespace sim