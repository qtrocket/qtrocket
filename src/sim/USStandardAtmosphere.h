#ifndef SIM_USSTANDARDATMOSPHERE_H
#define SIM_USSTANDARDATMOSPHERE_H

#include "AtmosphericModel.h"
#include "utils/BinMap.h"

namespace sim
{

class USStandardAtmosphere : public AtmosphericModel
{
public:
   USStandardAtmosphere();
   virtual ~USStandardAtmosphere();

   /**
    * @brief Get the density of the air at a given altitude above mean sea level
    * 
    * @param altitude the altitude above sea level
    * @return the density in kg/m^3
    */
   double getDensity(double altitude) override;
   double getPressure(double altitude) override;
   double getTemperature(double altitude) override;

private:


};

} // namespace sim
#endif // SIM_USSTANDARDATMOSPHERE_H