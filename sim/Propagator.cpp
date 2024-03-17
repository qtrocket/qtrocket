
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

Propagator::Propagator(std::shared_ptr<model::Rocket> r)
   : linearIntegrator(),
     //orientationIntegrator(),
     object(r),
     currentState(),
     nextState(),
     currentGravity(),
     currentWindSpeed(),
     saveStates(true),
     currentTime(0.0),
     timeStep(0.01),
     states()
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

//  std::function<std::pair<Quaternion, Quaternion>(Quaternion&, Quaternion&)> orientationODE =
//  [this](Quaternion& qOri, Quaternion& qRate) -> std::pair<Quaternion, Quaternion>
//  {
//     Quaternion dOri;
//     Quaternion dOriRate;

//     Matrix4
//  }

  saveStates = true;
}

Propagator::~Propagator()
{
}

void Propagator::runUntilTerminate()
{
   std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
   std::chrono::steady_clock::time_point endTime;

   currentState.position = {0.0, 0.0, 0.0};
   while(true)
   {

      // tempRes gets overwritten
      std::tie(nextState.position, nextState.velocity) = linearIntegrator->step(currentState.position, currentState.velocity);

      //tempRes = linearIntegrator->step(currentBodyState);

      
      if(saveStates)
      {
         states.push_back(std::make_pair(currentTime, nextState));
      }
      if(object->terminateCondition(currentTime))
         break;

      currentTime += timeStep;
      currentState = nextState;
   }
   endTime = std::chrono::steady_clock::now();

   std::stringstream duration;
   duration << "runUntilTerminate time (microseconds): ";
   duration << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
   utils::Logger::getInstance()->debug(duration.str());

}

double Propagator::getMass()
{
    return object->getMass(currentTime);
}

double Propagator::getForceX()
{
    return 0.0;
    //QtRocket* qtrocket = QtRocket::getInstance();
    //return (currentState.velocity[0] >= 0 ? -1.0 : 1.0) *  qtrocket->getEnvironment()->getAtmosphericModel()->getDensity(currentState.position[2])/ 2.0 * 0.008107 * object->getDragCoefficient() * currentState.velocity[0]* currentState.velocity[0];
}

double Propagator::getForceY()
{
    return 0.0;
    //QtRocket* qtrocket = QtRocket::getInstance();
    //return (currentState.velocity[1] >= 0 ? -1.0 : 1.0) * qtrocket->getEnvironment()->getAtmosphericModel()->getDensity(currentState.position[2]) / 2.0 * 0.008107 * object->getDragCoefficient() * currentState.velocity[1]* currentState.velocity[1];
}

double Propagator::getForceZ()
{
    return 0.0;
    //QtRocket* qtrocket = QtRocket::getInstance();
    //double gravity = (qtrocket->getEnvironment()->getGravityModel()->getAccel(currentState.position[0], currentState.position[1], currentState.position[2]))[2];
    //double airDrag = (currentState.velocity[2] >= 0 ? -1.0 : 1.0) * qtrocket->getEnvironment()->getAtmosphericModel()->getDensity(currentState.position[2]) / 2.0 * 0.008107 * object->getDragCoefficient() * currentState.velocity[2]* currentState.velocity[2];
    //double thrust  = object->getThrust(currentTime);
    //return gravity + airDrag + thrust;
}

double Propagator::getTorqueP() { return 0.0; }
double Propagator::getTorqueQ() { return 0.0; }
double Propagator::getTorqueR() { return 0.0; }

Vector3 Propagator::getCurrentGravity()
{
   auto gravityModel = QtRocket::getInstance()->getEnvironment()->getGravityModel();
   auto gravityAccel = gravityModel->getAccel(currentState.position[0],
                                              currentState.position[1],
                                              currentState.position[2]);
   Vector3 gravityVector{gravityAccel[0],
                         gravityAccel[1],
                         gravityAccel[2]};
   

   Quaternion q = currentState.orientation;

   Vector3 res = q * gravityVector;
   

   return Vector3{0.0, 0.0, 0.0};


}

} // namespace sim
