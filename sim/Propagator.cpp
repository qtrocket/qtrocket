#include "Propagator.h"

#include "sim/RK4Solver.h"

namespace sim {

Propagator::Propagator()
{

//   solver = std::make_unique<sim::DESolver>(
//            new(RK4Solver(/* xvel */ [this](double, double* x) -> double { return })))

   // The state vector has components of the form: (x, y, z, xdot, ydot, zdot)

   integrator = std::make_unique<sim::DESolver>(new RK4Solver(
      /* dvx/dt */ [this](double, double* ) -> double { return getForceX() / getMass(); },
      /* dx/dt  */ [this](double, double* s) -> double {return s[3]; },
      /* dvy/dt */ [this](double, double* ) -> double { return getForceY() / getMass() },
      /* dy/dt  */ [this](double, double* s) -> double {return s[4]; },
      /* dvz/dt */ [this](double, double* ) -> double { return getForceZ() / getMass() },
      /* dz/dt  */ [this](double, double* s) -> double {return s[5]; }));




}

} // namespace sim
