
/// \cond
// C headers
// C++ headers
#include <cmath>
#include <chrono>
#include <iostream>
#include <sstream>
#include <utility>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "Propagator.h"

#include "QtRocket.h"
#include "model/Rocket.h"
#include "sim/RK4Solver.h"
#include "utils/Logger.h"


namespace sim {

Propagator::Propagator(Rocket* r)
   : linearIntegrator(),
     //orientationIntegrator(),
     rocket(r)

{


   // This is a little strange, but I have to populate the integrator unique_ptr
   // with reset. make_unique() doesn't work because the compiler can't seem to
   // deduce the template parameters correctly, and I don't want to specify them
   // manually either. RK4Solver constructor is perfectly capable of deducing it's
   // template types, and it derives from DESolver, so we can just reset the unique_ptr
   // and pass it a freshly allocated RK4Solver pointer

   // The state vector has components of the form:
   linearIntegrator.reset(new RK4Solver(
      /* dx/dt  */ [](const std::vector<double>& s, double) -> double {return s[3]; },
      /* dy/dt  */ [](const std::vector<double>& s, double) -> double {return s[4]; },
      /* dz/dt  */ [](const std::vector<double>& s, double) -> double {return s[5]; },
      /* dvx/dt */ [this](const std::vector<double>&, double ) -> double { return getForceX() / getMass(); },
      /* dvy/dt */ [this](const std::vector<double>&, double ) -> double { return getForceY() / getMass(); },
      /* dvz/dt */ [this](const std::vector<double>&, double ) -> double { return getForceZ() / getMass(); }));
   linearIntegrator->setTimeStep(timeStep);
   
//   orientationIntegrator.reset(new RK4Solver(
//       /* dpitch/dt    */ [](double, const std::vector<double>& s) -> double { return s[3]; },
//       /* dyaw/dt      */ [](double, const std::vector<double>& s) -> double { return s[4]; },
//       /* droll/dt     */ [](double, const std::vector<double>& s) -> double { return s[5]; },
//       /* dpitchRate/dt */ [this](double, const std::vector<double>& s) -> double { return (getTorqueP() - s[1] * s[2] * (getIroll() - getIyaw())) / getIpitch(); },
//       /* dyawRate/dt   */ [this](double, const std::vector<double>& s) -> double { return (getTorqueQ() - s[0] * s[2] * (getIpitch() - getIroll())) / getIyaw(); },
//      /* drollRate/dt   */ [this](double, const std::vector<double>& s) -> double { return (getTorqueR() - s[0] * s[1] * (getIyaw() - getIpitch())) / getIroll(); }));

//   orientationIntegrator->setTimeStep(timeStep);

   saveStates = true;
}

Propagator::~Propagator()
{
}

void Propagator::runUntilTerminate()
{
   std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
   std::chrono::steady_clock::time_point endTime;

   while(true)
   {
      // tempRes gets overwritten
      linearIntegrator->step(currentState, tempRes);

      std::swap(currentState, tempRes);
      if(saveStates)
      {
         states.push_back(std::make_pair(currentTime, currentState));
      }
      if(rocket->terminateCondition(std::make_pair(currentTime, currentState)))
         break;

      currentTime += timeStep;
   }
   endTime = std::chrono::steady_clock::now();

   std::stringstream duration;
   duration << "runUntilTerminate time (microseconds): ";
   duration << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
   utils::Logger::getInstance()->debug(duration.str());

}

double Propagator::getMass()
{
    return rocket->getMass(currentTime);
}

double Propagator::getForceX()
{
    QtRocket* qtrocket = QtRocket::getInstance();
    return (currentState[3] >= 0 ? -1.0 : 1.0) *  qtrocket->getEnvironment()->getAtmosphericModel()->getDensity(currentState[2])/ 2.0 * 0.008107 * rocket->getDragCoefficient() * currentState[3]* currentState[3];
}

double Propagator::getForceY()
{
    QtRocket* qtrocket = QtRocket::getInstance();
    return (currentState[4] >= 0 ? -1.0 : 1.0) * qtrocket->getEnvironment()->getAtmosphericModel()->getDensity(currentState[2]) / 2.0 * 0.008107 * rocket->getDragCoefficient() * currentState[4]* currentState[4];
}

double Propagator::getForceZ()
{
    QtRocket* qtrocket = QtRocket::getInstance();
    double gravity = (qtrocket->getEnvironment()->getGravityModel()->getAccel(currentState[0], currentState[1], currentState[2]))[2];
    double airDrag = (currentState[5] >= 0 ? -1.0 : 1.0) * qtrocket->getEnvironment()->getAtmosphericModel()->getDensity(currentState[2]) / 2.0 * 0.008107 * rocket->getDragCoefficient() * currentState[5]* currentState[5];
    double thrust  = rocket->getThrust(currentTime);
    return gravity + airDrag + thrust;
}

double Propagator::getTorqueP() { return 0.0; }
double Propagator::getTorqueQ() { return 0.0; }
double Propagator::getTorqueR() { return 0.0; }

} // namespace sim
