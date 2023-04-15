#include "USStandardAtmosphere.h"

#include "utils/math/Constants.h"
#include <cmath>

using namespace utils::math;

namespace sim
{

// Populate static data
utils::BinMap initTemps()
{
   utils::BinMap map;
   map.insert(std::make_pair(0.0, 288.15));
   map.insert(std::make_pair(11000.0, 216.65));
   map.insert(std::make_pair(20000.0, 216.65));
   map.insert(std::make_pair(32000.0, 228.65));
   map.insert(std::make_pair(47000.0, 270.65));
   map.insert(std::make_pair(51000.0, 270.65));
   map.insert(std::make_pair(71000.0, 214.65));

   return map;

}

utils::BinMap initLapseRates()
{
   utils::BinMap map;
   map.insert(std::make_pair(0.0, 0.0065));
   map.insert(std::make_pair(11000.0, 0.0));
   map.insert(std::make_pair(20000.0, -0.001));
   map.insert(std::make_pair(32000.0, -0.0028));
   map.insert(std::make_pair(47000.0, 0.0));
   map.insert(std::make_pair(51000.0, 0.0028));
   map.insert(std::make_pair(71000.0, 0.002));

   return map;
}

utils::BinMap initDensities()
{
   utils::BinMap map;
   map.insert(std::make_pair(0.0, 1.225));
   map.insert(std::make_pair(11000.0, 0.36391));
   map.insert(std::make_pair(20000.0, 0.08803));
   map.insert(std::make_pair(32000.0, 0.01322));
   map.insert(std::make_pair(47000.0, 0.00143));
   map.insert(std::make_pair(51000.0, 0.00086));
   map.insert(std::make_pair(71000.0, 0.000064));

   return map;
}

utils::BinMap USStandardAtmosphere::temperatureLapseRate(initLapseRates());
utils::BinMap USStandardAtmosphere::standardTemperature(initTemps());
utils::BinMap USStandardAtmosphere::standardDensity(initDensities());



USStandardAtmosphere::USStandardAtmosphere()
{

}

USStandardAtmosphere::~USStandardAtmosphere()
{

}

double USStandardAtmosphere::getDensity(double altitude)
{
   if(temperatureLapseRate[altitude] == 0.0)
   {
      return standardDensity[altitude] * std::exp((-Constants::g0 * Constants::airMolarMass * (altitude - standardDensity.getBinBase(altitude)))
                                                  / (Constants::Rstar * standardTemperature[altitude]));

   }
   else
   {
      double base = standardTemperature[altitude] /
         (standardTemperature[altitude] + temperatureLapseRate[altitude] * (altitude - standardDensity.getBinBase(altitude)));
      
      double exponent = 1 + (Constants::g0 * Constants::airMolarMass) /
         (Constants::Rstar * temperatureLapseRate[altitude]);

      return standardDensity[altitude] * std::pow(base, exponent);

   }
}

double USStandardAtmosphere::getTemperature(double /*altitude*/)
{
   return 0.0;
}
double USStandardAtmosphere::getPressure(double /* altitude */)
{
   return 0.0;
}
} // namespace sim
