#ifndef SIM_USSTANDARDATMOSPHERE_H
#define SIM_USSTANDARDATMOSPHERE_H

// qtrocket headers
#include "sim/AtmosphericModel.h"
#include "utils/Bin.h"

namespace sim
{

class USStandardAtmosphere : public AtmosphericModel
{
public:
   USStandardAtmosphere();
   virtual ~USStandardAtmosphere();

   /// Air density (kg/m^3) at @p altitude above mean sea level.
   /// @todo Verify against the 1976 US Standard Atmosphere / NOAA paper.
   double getDensity(double altitude) override;
   double getPressure(double altitude) override;
   double getTemperature(double altitude) override;

   double getSpeedOfSound(double altitude) override;

   double getDynamicViscosity(double altitude) override;

private:
   static utils::Bin temperatureLapseRate;
   static utils::Bin standardTemperature;
   static utils::Bin standardDensity;
   static utils::Bin standardPressure;


};

} // namespace sim
#endif // SIM_USSTANDARDATMOSPHERE_H
