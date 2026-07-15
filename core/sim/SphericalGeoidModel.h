#ifndef SIM_SPHERICALGEOIDMODEL_H
#define SIM_SPHERICALGEOIDMODEL_H

// qtrocket headers
#include "GeoidModel.h"

namespace sim
{

/// Mean-radius geoid: returns the WGS84 mean of Earth's polar and equatorial radii.
class SphericalGeoidModel : public GeoidModel
{
public:
    SphericalGeoidModel();
    ~SphericalGeoidModel() override;

    double getGroundLevel(double latitude, double longitude) override;

};

} // namespace sim

#endif // SIM_SPHERICALGEOIDMODEL_H
