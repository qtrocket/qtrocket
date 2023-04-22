#ifndef ROCKET_H
#define ROCKET_H

/// \cond
// C headers
// C++ headers
#include <memory>
#include <utility> // std::move

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/Thrustcurve.h"
#include "sim/Propagator.h"

class Rocket
{
public:
   Rocket();

   void launch();
   const std::vector<std::pair<double, std::vector<double>>>& getStates() const { return propagator.getStates(); }

   void setInitialState(const std::vector<double>& initState) { propagator.setInitialState(initState); }

   double getMass() const { return mass; }
   void setMass(double m) { mass = m;}

   void setDragCoefficient(double d) { dragCoeff = d; }
   double getDragCoefficient() const { return dragCoeff; }

   double getThrust(double t);
   void setThrustCurve(const Thrustcurve& curve);

   bool terminateCondition(const std::pair<double, std::vector<double>>& cond);

   void setName(const std::string& n) { name = n; }
private:

   std::string name;
   sim::Propagator propagator;
   double dragCoeff;
   double mass;

   Thrustcurve tc;

};

#endif // ROCKET_H
