#ifndef ROCKET_H
#define ROCKET_H

#include "sim/Propagator.h"
#include "model/Thrustcurve.h"

#include <utility> // std::move
#include <memory>

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
private:

   sim::Propagator propagator;
   double dragCoeff;
   double mass;

   Thrustcurve tc;

};

#endif // ROCKET_H
