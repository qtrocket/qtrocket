#ifndef SIM_SPHERICALGEOIDMODEL_H
#define SIM_SPHERICALGEOIDMODEL_H

// qtrocket headers
#include "GeoidModel.h"

namespace sim
{

/**
 * @brief The SphericalGeoidModel returns the average of the polar radius and equatorial
 *        radius of the Earth, based on WGS84
 * 
 */

class SphericalGeoidModel : public GeoidModel
{
public:
   SphericalGeoidModel();
   virtual ~SphericalGeoidModel();

   double getGroundLevel(double latitude, double longitude) override;

};

} // namespace sim

#endif // SIM_SPHERICALGEOIDMODEL_H
