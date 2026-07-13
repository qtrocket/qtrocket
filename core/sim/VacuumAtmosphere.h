#ifndef SIM_VACUUMATMOSPHERE_H
#define SIM_VACUUMATMOSPHERE_H

// qtrocket headers
#include "AtmosphericModel.h"

namespace sim {

/// A vacuum: every property is 0, so drag vanishes and the flight reduces to thrust + gravity.
/// Useful as a baseline, e.g. isolating integrator behavior from the aero model.
class VacuumAtmosphere : public AtmosphericModel
{
public:
    VacuumAtmosphere() {}
    virtual ~VacuumAtmosphere() {}

    double getDensity(double) override { return 0.0; }
    double getPressure(double) override { return 0.0; }
    double getTemperature(double) override { return 0.0; }

    // No medium -> sound doesn't propagate. Callers computing a Mach number must guard against 0.
    double getSpeedOfSound(double) override { return 0.0; }

    double getDynamicViscosity(double) override { return 0.0; }
};

} // namespace sim

#endif // SIM_VACUUMATMOSPHERE_H
