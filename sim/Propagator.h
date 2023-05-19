#ifndef SIM_PROPAGATOR_H
#define SIM_PROPAGATOR_H

/// \cond
// C headers
// C++ headers
#include <memory>
#include <vector>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "sim/DESolver.h"
#include "utils/math/MathTypes.h"
#include "sim/StateData.h"


// Forward declare
class Rocket;
class QtRocket;

namespace sim
{

class Propagator
{
public:
    Propagator(std::shared_ptr<Rocket> r);
    ~Propagator();

    void setInitialState(const std::vector<double>& initialState)
    {
       for(std::size_t i = 0; i < initialState.size(); ++i)
       {
           currentBodyState[i] = initialState[i];
       }

    }

    const Vector6& getCurrentState() const
    {
       return currentBodyState;
    }

    void runUntilTerminate();

    void retainStates(bool s)
    {
       saveStates = s;
    }

    const std::vector<std::pair<double, Vector6>>& getStates() const { return states; }

    void clearStates() { states.clear(); }
    void setCurrentTime(double t) { currentTime = t; }
    void setTimeStep(double ts) { timeStep = ts; }
    void setSaveStats(bool s) { saveStates = s; }

private:
   double getMass();
   double getForceX();
   double getForceY();
   double getForceZ();

   double getTorqueP();
   double getTorqueQ();
   double getTorqueR();

   double getIpitch() { return 1.0; }
   double getIyaw()   { return 1.0; }
   double getIroll()  { return 1.0; }

   std::unique_ptr<sim::RK4Solver<Vector3>> linearIntegrator;
   std::unique_ptr<sim::RK4Solver<Quaternion>> orientationIntegrator;

   std::shared_ptr<Rocket> rocket;

   StateData worldFrameState;
   //StateData bodyFrameState;
   Vector3 currentBodyPosition{0.0, 0.0, 0.0};
   Vector3 currentBodyVelocity{0.0, 0.0, 0.0};
   Vector3 nextBodyPosition{0.0, 0.0, 0.0};
   Vector3 nextBodyVelocity{0.0, 0.0, 0.0};

   std::vector<double> currentWorldState{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

   Vector3 currentGravity{0.0, 0.0, 0.0};
   Vector3 currentWindSpeed{0.0, 0.0, 0.0};
   

   // orientation vectors in the form (yawDot, pitchDot, rollDot, q1, q2, q3, q4)
   Quaternion currentOrientation{0.0, 0.0, 0.0, 0.0};
   Quaternion currentOrientationRate{0.0, 0.0, 0.0, 0.0};
   Quaternion nextOrientation{0.0, 0.0, 0.0, 0.0};
   Quaternion nextOrientationRate{0.0, 0.0, 0.0, 0.0};
   bool saveStates{true};
   double currentTime{0.0};
   double timeStep{0.01};
   std::vector<std::pair<double, Vector6>> states;

   Vector3 getCurrentGravity();
};

} // namespace sim

#endif // SIM_PROPAGATOR_H
