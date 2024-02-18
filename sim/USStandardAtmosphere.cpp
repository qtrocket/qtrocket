
/// \cond
// C headers
// C++ headers
#include <cmath>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "sim/USStandardAtmosphere.h"
#include "utils/math/Constants.h"
#include "utils/math/UtilityMathFunctions.h"

using namespace utils::math;

namespace sim
{

// Populate static data
utils::Bin initTemps()
{
   utils::Bin map;
   map.insert(std::make_pair(0.0, 288.15));
   map.insert(std::make_pair(11000.0, 216.65));
   map.insert(std::make_pair(20000.0, 216.65));
   map.insert(std::make_pair(32000.0, 228.65));
   map.insert(std::make_pair(47000.0, 270.65));
   map.insert(std::make_pair(51000.0, 270.65));
   map.insert(std::make_pair(71000.0, 214.65));

   return map;

}

utils::Bin initLapseRates()
{
   utils::Bin map;
   map.insert(std::make_pair(0.0, 0.0065));
   map.insert(std::make_pair(11000.0, 0.0));
   map.insert(std::make_pair(20000.0, -0.001));
   map.insert(std::make_pair(32000.0, -0.0028));
   map.insert(std::make_pair(47000.0, 0.0));
   map.insert(std::make_pair(51000.0, 0.0028));
   map.insert(std::make_pair(71000.0, 0.002));

   return map;
}

utils::Bin initDensities()
{
   utils::Bin map;
   map.insert(std::make_pair(0.0, 1.225));
   map.insert(std::make_pair(11000.0, 0.36391));
   map.insert(std::make_pair(20000.0, 0.08803));
   map.insert(std::make_pair(32000.0, 0.01322));
   map.insert(std::make_pair(47000.0, 0.00143));
   map.insert(std::make_pair(51000.0, 0.00086));
   map.insert(std::make_pair(71000.0, 0.000064));

   return map;
}

utils::Bin initPressures()
{
   utils::Bin map;
   map.insert(std::make_pair(0.0, 101325));
   map.insert(std::make_pair(11000.0, 22632.1));
   map.insert(std::make_pair(20000.0, 5474.89));
   map.insert(std::make_pair(32000.0, 868.02));
   map.insert(std::make_pair(47000.0, 110.91));
   map.insert(std::make_pair(51000.0, 66.94));
   map.insert(std::make_pair(71000.0, 3.96));

   return map;
}

utils::Bin USStandardAtmosphere::temperatureLapseRate(initLapseRates());
utils::Bin USStandardAtmosphere::standardTemperature(initTemps());
utils::Bin USStandardAtmosphere::standardDensity(initDensities());
utils::Bin USStandardAtmosphere::standardPressure(initPressures());

USStandardAtmosphere::USStandardAtmosphere()
{

}

USStandardAtmosphere::~USStandardAtmosphere()
{

}

double USStandardAtmosphere::getDensity(double altitude)
{
   if(utils::math::floatingPointEqual(temperatureLapseRate[altitude], 0.0))
   {
      return standardDensity[altitude] * std::exp(
            (-Constants::g0 * Constants::airMolarMass * (altitude - standardDensity.getBinBase(altitude)))
                                                  / (Constants::Rstar * standardTemperature[altitude]));

   }
   else
   {
      double base = (standardTemperature[altitude] - temperatureLapseRate[altitude] *
                    (altitude - standardDensity.getBinBase(altitude))) / standardTemperature[altitude];
      
      double exponent = (Constants::g0 * Constants::airMolarMass) /
         (Constants::Rstar * temperatureLapseRate[altitude]) - 1.0;

      return standardDensity[altitude] * std::pow(base, exponent);
   }
}

double USStandardAtmosphere::getTemperature(double altitude)
{
   double baseTemp = standardTemperature[altitude];
   double baseAltitude = standardTemperature.getBinBase(altitude);
   return baseTemp - (altitude - baseAltitude) * temperatureLapseRate[altitude];

}
double USStandardAtmosphere::getPressure(double altitude)
{
   if(utils::math::floatingPointEqual(temperatureLapseRate[altitude], 0.0))
   {
      return standardPressure[altitude] * std::exp(
            (-Constants::g0 * Constants::airMolarMass * (altitude - standardPressure.getBinBase(altitude)))
                                                  / (Constants::Rstar * standardTemperature[altitude]));

   }
   else
   {
      double base = (standardTemperature[altitude] - temperatureLapseRate[altitude] *
                    (altitude - standardPressure.getBinBase(altitude))) / standardTemperature[altitude];
      
      double exponent = (Constants::g0 * Constants::airMolarMass) /
         (Constants::Rstar * temperatureLapseRate[altitude]);

      return standardPressure[altitude] * std::pow(base, exponent);
   }

}

double USStandardAtmosphere::getSpeedOfSound(double altitude)
{
   return std::sqrt( (Constants::gamma * Constants::Rstar * getTemperature(altitude))
                     /
                     Constants::airMolarMass);
}

double USStandardAtmosphere::getDynamicViscosity(double altitude)
{
   double temperature = getTemperature(altitude);
   return (Constants::beta * std::pow(temperature, 1.5)) / ( temperature + Constants::S);
}
} // namespace sim
