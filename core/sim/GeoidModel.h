#ifndef SIM_GEOIDMODEL_H
#define SIM_GEOIDMODEL_H

namespace sim
{

/// Earth's reference ellipsoid: gives the ground level's distance from Earth's center.
class GeoidModel
{
public:
   GeoidModel() {}
   virtual ~GeoidModel() {}

   virtual double getGroundLevel(double latitude, double longitude) = 0;

};

} // namespace sim

#endif // SIM_GEOIDMODEL_H