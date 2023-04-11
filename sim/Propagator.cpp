#include <iostream>
#include "Propagator.h"

#include "sim/RK4Solver.h"
#include "model/Rocket.h"

#include <utility>
#include <QTextStream>

namespace sim {

Propagator::Propagator(Rocket* r)
   : integrator(),
     rocket(r)

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
   saveStates = true;
}

Propagator::~Propagator()
{
}

void Propagator::runUntilTerminate()
{

    QTextStream out(stdout);
       std::size_t j = 0;
   while(true && j < 100000)
   {
      // nextState gets overwritten
      integrator->step(currentTime, currentState, tempRes);
      /*
      std::size_t size = currentState.size();
      for(size_t i = 0; i < size; ++i)
      {
          currentState[i] = tempRes[i];
          tempRes[i] = 0;
      }
      */

      std::swap(currentState, nextState);
      if(saveStates)
      {
         states.push_back(currentState);
      }
      out << currentTime << ": ("
          << currentState[0] << ", "
          << currentState[1] << ", "
          << currentState[2] << ", "
          << currentState[3] << ", "
          << currentState[4] << ", "
          << currentState[5] << ")\n";
      if(currentState[1] < 0.0)
         break;

       j++;
      currentTime += timeStep;
   }
}

double Propagator::getMass()
{
    return rocket->getMass();
}

double Propagator::getForceX() { return -1.225 / 2.0 * 0.008107 * rocket->getDragCoefficient() * currentState[3]* currentState[3]; }
double Propagator::getForceY() { return -1.225 / 2.0 * 0.008107 * rocket->getDragCoefficient() * currentState[4]* currentState[4] -9.8; }
double Propagator::getForceZ() { return 0; }

double Propagator::getTorqueP() { return 0.0; }
double Propagator::getTorqueQ() { return 0.0; }
double Propagator::getTorqueR() { return 0.0; }

} // namespace sim
