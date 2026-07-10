#ifndef SIM_CONSTANTATMOSPHERE_H
#define SIM_CONSTANTATMOSPHERE_H

// qtrocket headers
#include "AtmosphericModel.h"

namespace sim {

class ConstantAtmosphere : public AtmosphericModel
{
public:
   ConstantAtmosphere() {}
   virtual ~ConstantAtmosphere() {}

   double getDensity(double) override { return 1.225; }
   double getPressure(double) override { return 101325.0; }
   double getTemperature(double) override { return 288.15; }

   double getSpeedOfSound(double) override { return 340.294; }

   double getDynamicViscosity(double) override { return 1.78938e-5; }
};

} // namespace sim

#endif // SIM_CONSTANTATMOSPHERE_H
