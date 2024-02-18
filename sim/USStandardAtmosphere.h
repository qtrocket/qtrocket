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

   /**
    * @brief Get the density of the air at a given altitude above mean sea level
    *        This is overly simplistic and wrong implementation.
    * 
    * @todo Fix this implementation. See the 1976 NOAA paper for the right way to
    *       do it
    * 
    * @param altitude the altitude above sea level
    * @return the density in kg/m^3
    */
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
