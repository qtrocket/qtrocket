
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

Propagator::Propagator(std::shared_ptr<Rocket> r)
   : linearIntegrator(),
     orientationIntegrator(),
     rocket(r),
     currentBodyState(),
     worldFrameState()
{


   // This is a little strange, but I have to populate the integrator unique_ptr
   // with reset. make_unique() doesn't work because the compiler can't seem to
   // deduce the template parameters correctly, and I don't want to specify them
   // manually either. RK4Solver constructor is perfectly capable of deducing it's
   // template types, and it derives from DESolver, so we can just reset the unique_ptr
   // and pass it a freshly allocated RK4Solver pointer

   // The state vector has components of the form:

   // Linear velocity and acceleration
   std::function<std::pair<Vector3, Vector3>(Vector3&, Vector3&)> linearODE = [this](Vector3& state, Vector3& rate) -> std::pair<Vector3, Vector3>
   {
      Vector3 dPosition;
      Vector3 dVelocity;
      // dx/dt
      dPosition[0] = rate[0];

      // dy/dt
      dPosition[1] = rate[1];

      // dz/dt
      dPosition[2] = rate[2];

      // dvx/dt
      dVelocity[0] = getForceX() / getMass();

      // dvy/dt
      dVelocity[1] = getForceY() / getMass();

      // dvz/dt
      dVelocity[2] = getForceZ() / getMass();

      return std::make_pair(dPosition, dVelocity);

   };

   linearIntegrator.reset(new RK4Solver<Vector3>(linearODE));
   linearIntegrator->setTimeStep(timeStep);
   
   // This is pure quaternion
   // This formula is taken from https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-attitudetran
   //
   // Convention for the quaternions is (vector, scalar). So q4 is scalar term
   //
   // q1Dot = 1/2 * [(q4*omega1) - (q3*omega2) + (q2*omega3)]
   // q2Dot = 1/2 * [(q3*omega1) + (q4*omega2) - (q1*omega3)]
   // q3Dot = 1/2 * [(-q2*omega1) + (q1*omega2) + (q4*omega3)]
   // q4Dot = -1/2 *[(q1*omega1) + (q2*omega2) + (q3*omega3)]
   //
   // omega1 = yawRate
   // omega2 = pitchRate
   // omega3 = rollRate
   //
//   orientationIntegrator.reset(new RK4Solver(
//       /* dyawRate/dt    */ [this](const std::vector<double>& s, double) -> double
//       { return getTorqueP() / getIyaw(); },
//       /* dpitchRate/dt  */ [this](const std::vector<double>& s, double) -> double
//       { return getTorqueQ() / getIpitch(); },
//       /* drollRate/dt   */ [this](const std::vector<double>& s, double) -> double
//       { return getTorqueR() / getIroll(); },
//       /* q1Dot */ [](const std::vector<double>& s, double) -> double
//       {
//           double retVal = s[6]*s[0] - s[5]*s[1] + s[4]*s[2];
//           return 0.5 * retVal;
//       },
//       /* q2Dot */ [](const std::vector<double>& s, double) -> double
//       {
//           double retVal = s[5]*s[0] + s[6]*s[1] - s[3]*s[2];
//           return 0.5 * retVal;
//       },
//       /* q3Dot */ [](const std::vector<double>& s, double) -> double
//       {
//           double retVal = -s[4]*s[0] + s[3]*s[1] + s[6]*s[2];
//           return 0.5 * retVal;
//       },
//       /* q4Dot */ [](const std::vector<double>& s, double) -> double
//       {
//           double retVal = s[3]*s[0] + s[4]*s[1] + s[5]*s[2];
//           return -0.5 * retVal;
//       }));
//   orientationIntegrator->setTimeStep(timeStep);

   std::function<std::pair<Quaternion, Quaternion>(Quaternion&, Quaternion&)> orientationODE =
   [this](Quaternion& qOri, Quaternion& qRate) -> std::pair<Quaternion, Quaternion>
   {
      Quaternion dOri;
      Quaternion dOriRate;

      Matrix4
   }

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

      // Reset the body frame positions to zero since the origin of the body frame is the CM
      currentBodyState[0] = 0.0;
      currentBodyState[1] = 0.0;
      currentBodyState[2] = 0.0;

      // tempRes gets overwritten
      tempRes = linearIntegrator->step(currentBodyState);

      std::swap(currentBodyState, tempRes);
      
      if(saveStates)
      {
         states.push_back(std::make_pair(currentTime, currentBodyState));
      }
      if(rocket->terminateCondition(std::make_pair(currentTime, currentBodyState)))
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
    return (currentBodyState[3] >= 0 ? -1.0 : 1.0) *  qtrocket->getEnvironment()->getAtmosphericModel()->getDensity(currentBodyState[2])/ 2.0 * 0.008107 * rocket->getDragCoefficient() * currentBodyState[3]* currentBodyState[3];
}

double Propagator::getForceY()
{
    QtRocket* qtrocket = QtRocket::getInstance();
    return (currentBodyState[4] >= 0 ? -1.0 : 1.0) * qtrocket->getEnvironment()->getAtmosphericModel()->getDensity(currentBodyState[2]) / 2.0 * 0.008107 * rocket->getDragCoefficient() * currentBodyState[4]* currentBodyState[4];
}

double Propagator::getForceZ()
{
    QtRocket* qtrocket = QtRocket::getInstance();
    double gravity = (qtrocket->getEnvironment()->getGravityModel()->getAccel(currentBodyState[0], currentBodyState[1], currentBodyState[2]))[2];
    double airDrag = (currentBodyState[5] >= 0 ? -1.0 : 1.0) * qtrocket->getEnvironment()->getAtmosphericModel()->getDensity(currentBodyState[2]) / 2.0 * 0.008107 * rocket->getDragCoefficient() * currentBodyState[5]* currentBodyState[5];
    double thrust  = rocket->getThrust(currentTime);
    return gravity + airDrag + thrust;
}

double Propagator::getTorqueP() { return 0.0; }
double Propagator::getTorqueQ() { return 0.0; }
double Propagator::getTorqueR() { return 0.0; }

Vector3 Propagator::getCurrentGravity()
{
   auto gravityModel = QtRocket::getInstance()->getEnvironment()->getGravityModel();
   auto gravityAccel = gravityModel->getAccel(currentBodyState[0],
                                              currentBodyState[1],
                                              currentBodyState[2]);
   Vector3 gravityVector{gravityAccel[0],
                         gravityAccel[1],
                         gravityAccel[2]};
   

   Quaternion q{currentOrientation[3],
                currentOrientation[4],
                currentOrientation[5],
                currentOrientation[6]};

   Vector3 res = q * gravityVector;
   

   return Vector3{0.0, 0.0, 0.0};


}

} // namespace sim
