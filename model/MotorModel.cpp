
/// \cond
// C headers
// C++ headers
// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"
#include "utils/math/Constants.h"
#include "utils/math/UtilityMathFunctions.h"
#include "utils/Logger.h"

namespace model
{

MotorModel::MotorModel()
{

}

MotorModel::~MotorModel()
{

}

double MotorModel::getMass(double simTime) const
{
   // the current mass is the emptyMass + the current prop mass

   // If ignition hasn't occurred, return the totalMass
   if(!ignitionOccurred)
   {
      return data.totalWeight;
   }
   else if(simTime - ignitionTime <= data.burnTime)
   {
      double thrustTime = simTime - ignitionTime;
      // Find the right interval in the massCurve
      auto i = massCurve.cbegin();
      while(i->first <= thrustTime)
      {
         // If thrustTime is equal to a data point that we have, then just return
         // the mass at that time. Otherwise it fell between two points and we
         // will interpolate
         if(utils::math::floatingPointEqual(i->first, thrustTime))
         {
            // return empty mass + remaining propellant mass
            return emptyMass + i->second;
         }
         else
         {
            i++;
         }
      }
      // linearly interpolate the propellant mass. Then return the empty mass + remaining prop mass
      double tStart = std::prev(i)->first;
      double tEnd = i->first;
      double propMassStart = std::prev(i)->second;
      double propMassEnd   = i->second;
      double slope = (propMassEnd - propMassStart) / (tEnd - tStart);
      double currentMass = emptyMass + propMassStart + (thrustTime - tStart) * slope;
      utils::Logger::getInstance()->info("simTime: " + std::to_string(simTime) + ": motor mass: " + std::to_string(currentMass));
      return currentMass;

   }
   // motor has burned out
   else
   {
      return emptyMass;
   }
}

double MotorModel::getThrust(double simTime)
{

   if(simTime > thrust.getMaxTime() + ignitionTime)
   {
      if(!burnOutOccurred)
      {
         utils::Logger::getInstance()->info("motor burnout occurred: " + std::to_string(simTime));
         burnOutOccurred = true;
      }
      return 0.0;
   }
   utils::Logger::getInstance()->info("simTime: " + std::to_string(simTime) + ": thrust: " + std::to_string(thrust.getThrust(simTime)));
   return thrust.getThrust(simTime);
}

void MotorModel::setMetaData(const MetaData& md)
{
   data = md;
   computeMassCurve();
}

void MotorModel::moveMetaData(MetaData&& md)
{
   data = std::move(md);
   computeMassCurve();
}

void MotorModel::computeMassCurve()
{
   emptyMass = data.totalWeight - data.propWeight;

   // Calculate the Isp for the motor, as we'll need this for the computing the mass flow rate.
   // This will be the total impulse in Newton-seconds over
   // the propellant weight. The prop mass is in grams, hence the division by 1000.0 to get kg
   isp = data.totalImpulse / (utils::math::Constants::g0 * data.propWeight / 1000.0);

   // Precompute the mass curve. Having this precomputed will ensure multiple calls to getMass()
   // or getThrust() during the same time step don't accidentally decrement the mass multiple times.
   // Having a lookup table will ensure consistent mass values, as well as speed up the simulation,
   // just at the cost of some extra space

   // Most motor data in the RASP format has a limitation of 32 data points. We're not going to
   // match that, so we can pick whatever we want and just interpolate values. We can have 128
   // for example

   massCurve.reserve(128);
   double timeStep = data.burnTime / 127.0;
   double t = 0.0;
   double propMass{data.propWeight};
   for(std::size_t i = 0; i < 127; ++i)
   {
      massCurve.push_back(std::make_pair(t + i*timeStep, propMass));
      propMass -= thrust.getThrust(t + i*timeStep) * timeStep * data.propWeight / data.totalImpulse;
   }
   massCurve.push_back(std::make_pair(data.burnTime, 0.0));
}
} // namespace model
