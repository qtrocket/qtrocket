#ifndef SIM_VACUUMATMOSPHERE_H
#define SIM_VACUUMATMOSPHERE_H

// qtrocket headers
#include "AtmosphericModel.h"

namespace sim {

/**
 * @brief A vacuum: zero air. Every property returns 0, so any density-driven
 *        aerodynamic force (drag) evaluates to zero. This reduces the flight
 *        model to thrust + gravity (a point mass in vacuum), which is useful as
 *        a baseline -- e.g. for isolating integrator behavior from the
 *        aerodynamic model.
 */
class VacuumAtmosphere : public AtmosphericModel
{
public:
   VacuumAtmosphere() {}
   virtual ~VacuumAtmosphere() {}

   double getDensity(double) override { return 0.0; }
   double getPressure(double) override { return 0.0; }
   double getTemperature(double) override { return 0.0; }

   // No medium -> sound does not propagate. Returns 0; callers that compute a
   // Mach number must guard against a zero speed of sound (none do today).
   double getSpeedOfSound(double) override { return 0.0; }

   double getDynamicViscosity(double) override { return 0.0; }
};

} // namespace sim

#endif // SIM_VACUUMATMOSPHERE_H
