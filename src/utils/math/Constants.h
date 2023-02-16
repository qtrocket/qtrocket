#ifndef UTILS_MATH_CONSTANTS_H
#define UTILS_MATH_CONSTANTS_H

namespace utils::math
{

namespace Constants
{
   constexpr double Rstar = 8.3144598;
   constexpr const double g0 = 9.80665;
   constexpr const double airMolarMass = 0.0289644;
   constexpr const double standardTemperature = 288.15;
   constexpr const double standardDensity = 1.2250;
   constexpr const double meanEarthRadiusWGS84 = 6371008.8;

   constexpr const long double earthGM = 398600441800000.0;
   constexpr const double earthGM_km = 398600.4418;

};

} // namespace utils::math
#endif // UTILS_MATH_CONSTANTS_H