#ifndef SIM_GEOIDMODEL_H
#define SIM_GEOIDMODEL_H

namespace sim
{

/**
 * @brief The GeoidModel represents the physical ellipsoid of the earth. It is used
 *        to determin the distance of the local ground level from the center of mass
 *        of the earth.
 * 
 */
class GeoidModel
{
public:
   GeoidModel() {}
   virtual ~GeoidModel() {}

   virtual double getGroundLevel(double latitude, double longitude) = 0;

};

} // namespace sim

#endif // SIM_GEOIDMODEL_H