#ifndef ROCKET_H
#define ROCKET_H

#include "sim/Propagator.h"

#include <utility> // std::move
#include <memory>

class Rocket
{
public:
   Rocket();

   void launch();
   const std::vector<std::vector<double>>& getStates() const { return propagator.getStates(); }

   void setInitialState(const std::vector<double>& initState) { propagator.setInitialState(initState); }

   double getMass() const { return mass; }
   void setMass(double m) { mass = m;}

   void setDragCoefficient(double d) { dragCoeff = d; }
   double getDragCoefficient() const { return dragCoeff; }
private:

   sim::Propagator propagator;
   double dragCoeff;
   double mass;
};

#endif // ROCKET_H
