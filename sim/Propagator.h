#ifndef SIM_PROPAGATOR_H
#define SIM_PROPAGATOR_H

#include "sim/DESolver.h"

#include <memory>

namespace sim
{

class Propagator
{
public:
    Propagator();

   double getForceX();
   double getForceY();
   double getForceZ();

   double getTorqueP();
   double getTorqueQ();
   double getTorqueR();

   double getMass();

private:

   std::unique_ptr<sim::DESolver> integrator;

   double currentState[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   double nextState[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

} // namespace sim

#endif // SIM_PROPAGATOR_H
