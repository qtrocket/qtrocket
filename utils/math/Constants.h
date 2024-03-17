#ifndef UTILS_MATH_CONSTANTS_H
#define UTILS_MATH_CONSTANTS_H

namespace utils::math
{

namespace Constants
{
   constexpr double Rstar = 8.31432;
   constexpr const double g0 = 9.80665;
   constexpr const double airMolarMass = 0.0289644;

   // gamma is the ratio of the specific heat of air at constant pressure to that at
   // constant volume. Used in computing the speed of sound
   constexpr const double gamma = 1.4;

   // beta is used in calculating the dynamic viscosity of air. Based on the 1976 US Standard
   // Atmosphere report, it is empirically measured to be:
   constexpr const double beta = 1.458e-6;
   // Sutherland's constant
   constexpr const double S = 110.4;
   constexpr const double standardTemperature = 288.15;
   constexpr const double standardDensity = 1.2250;
   constexpr const double meanEarthRadiusWGS84 = 6371008.8;

   constexpr const long double earthGM = 398600441800000.0;
   constexpr const double earthGM_km = 398600.4418;

};

} // namespace utils::math
#endif // UTILS_MATH_CONSTANTS_H
