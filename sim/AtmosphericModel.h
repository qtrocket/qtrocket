#ifndef SIM_ATMOSPHERICMODEL_H
#define SIM_ATMOSPHERICMODEL_H

namespace sim
{

class AtmosphericModel
{
public:
   AtmosphericModel() {}
   virtual ~AtmosphericModel() {}

   virtual double getDensity(double altitude) = 0;
   virtual double getPressure(double altitude) = 0;
   virtual double getTemperature(double altitude) = 0;

   virtual double getSpeedOfSound(double altitude) = 0;
   virtual double getDynamicViscosity(double altitude) = 0;

};

} // namespace sim
#endif // SIM_ATMOSPHERICMODEL_H
