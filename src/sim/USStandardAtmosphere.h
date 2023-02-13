#ifndef SIM_USSTANDARDATMOSPHERE_H
#define SIM_USSTANDARDATMOSPHERE_H

#include "AtmosphericModel.h"

namespace sim
{

class USStandardAtmosphere : public AtmosphericModel
{
public:
   USStandardAtmosphere();
   virtual ~USStandardAtmosphere();

   double getDensity(double altitude) override;
   double getPressure(double altitude) override;
   double getTemperature(double altitude) override;

private:


};

} // namespace sim
#endif // SIM_USSTANDARDATMOSPHERE_H