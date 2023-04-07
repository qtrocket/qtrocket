#include "Propagator.h"

#include "sim/RK4Solver.h"

#include <utility>

namespace sim {

Propagator::Propagator()
   : integrator()

{


   // This is a little strange, but I have to populate the integrator unique_ptr
   // with reset. make_unique() doesn't work because the compiler can't seem to
   // deduce the template parameters correctly, and I don't want to specify them
   // manually either. RK4Solver constructor is perfectly capable of deducing it's
   // template types, and it derives from DESolver, so we can just reset the unique_ptr
   // and pass it a freshly allocated RK4Solver pointer

   // The state vector has components of the form: (x, y, z, xdot, ydot, zdot)
   integrator.reset(new RK4Solver(
      /* dx/dt  */ [](double, const std::vector<double>& s) -> double {return s[3]; },
      /* dy/dt  */ [](double, const std::vector<double>& s) -> double {return s[4]; },
      /* dz/dt  */ [](double, const std::vector<double>& s) -> double {return s[5]; },
      /* dvx/dt */ [this](double, const std::vector<double>& ) -> double { return getForceX() / getMass(); },
      /* dvy/dt */ [this](double, const std::vector<double>& ) -> double { return getForceY() / getMass(); },
      /* dvz/dt */ [this](double, const std::vector<double>& ) -> double { return getForceZ() / getMass(); }));


   integrator->setTimeStep(timeStep);
}

Propagator::~Propagator()
{
}

void Propagator::runUntilTerminate()
{
   while(true)
   {
      // nextState gets overwritten
      integrator->step(currentTime, currentState, nextState);
      std::swap(currentState, nextState);
      if(saveStates)
      {
         states.push_back(currentState);
      }
      if(currentState[1] < 0.0)
         break;

      currentTime += timeStep;
   }
}

} // namespace sim
