
/// \cond
// C headers
// C++ headers
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
   : integrator(),
     rocket(r),
     qtrocket(QtRocket::getInstance())

{


   // This is a little strange, but I have to populate the integrator unique_ptr
   // with reset. make_unique() doesn't work because the compiler can't seem to
   // deduce the template parameters correctly, and I don't want to specify them
   // manually either. RK4Solver constructor is perfectly capable of deducing it's
   // template types, and it derives from DESolver, so we can just reset the unique_ptr
   // and pass it a freshly allocated RK4Solver pointer

   // The state vector has components of the form:
   // (x, y, z, xdot, ydot, zdot, pitch, yaw, roll, pitchRate, yawRate, rollRate)
   integrator.reset(new RK4Solver(
      /* dx/dt  */ [](double, const std::vector<double>& s) -> double {return s[3]; },
      /* dy/dt  */ [](double, const std::vector<double>& s) -> double {return s[4]; },
      /* dz/dt  */ [](double, const std::vector<double>& s) -> double {return s[5]; },
      /* dvx/dt */ [this](double, const std::vector<double>& ) -> double { return getForceX() / getMass(); },
      /* dvy/dt */ [this](double, const std::vector<double>& ) -> double { return getForceY() / getMass(); },
      /* dvz/dt */ [this](double, const std::vector<double>& ) -> double { return getForceZ() / getMass(); },
       /* dpitch/dt    */ [this](double, const std::vector<double>& s) -> double { return s[9]; },
       /* dyaw/dt      */ [this](double, const std::vector<double>& s) -> double { return s[10]; },
       /* droll/dt     */ [this](double, const std::vector<double>& s) -> double { return s[11]; },
       /* dpitchRate/dt */ [this](double, const std::vector<double>& s) -> double { (getTorqueP() - s[7] * s[8] * (getIroll() - getIyaw())) / getIpitch(); },
       /* dyawRate/dt   */ [this](double, const std::vector<double>& s) -> double { (getTorqueQ() - s[6] * s[9] * (getIpitch() - getIroll())) / getIyaw(); },
      /* drollRate/dt   */ [this](double, const std::vector<double>& s) -> double { (getTorqueR() - s[6] * s[7] * (getIyaw() - getIpitch())) / getIroll(); }));


   integrator->setTimeStep(timeStep);
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
      integrator->step(currentTime, currentState, tempRes);

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
    return rocket->getMass();
}

double Propagator::getForceX()
{
    return - qtrocket->getAtmosphereModel()->getDensity(currentState[3])/ 2.0 * 0.008107 * rocket->getDragCoefficient() * currentState[3]* currentState[3];
}

double Propagator::getForceY()
{
    return -qtrocket->getAtmosphereModel()->getDensity(currentState[3]) / 2.0 * 0.008107 * rocket->getDragCoefficient() * currentState[4]* currentState[4];
}

double Propagator::getForceZ()
{
    double gravity = (qtrocket->getGravityModel()->getAccel(currentState[0], currentState[1], currentState[2])).x3;
    double airDrag = -qtrocket->getAtmosphereModel()->getDensity(currentState[3]) / 2.0 * 0.008107 * rocket->getDragCoefficient() * currentState[5]* currentState[5];
    double thrust  = rocket->getThrust(currentTime);
    return gravity + airDrag + thrust;
}

double Propagator::getTorqueP() { return 0.0; }
double Propagator::getTorqueQ() { return 0.0; }
double Propagator::getTorqueR() { return 0.0; }

} // namespace sim
