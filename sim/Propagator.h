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
#include "sim/RK4Solver.h"
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

    void setInitialState(const StateData& initialState)
    {
           currentState = initialState;
    }

    const StateData& getCurrentState() const
    {
       return currentState;
    }

    void runUntilTerminate();

    void retainStates(bool s)
    {
       saveStates = s;
    }

    const std::vector<std::pair<double, StateData>>& getStates() const { return states; }

    void clearStates() { states.clear(); }
    void setCurrentTime(double t) { currentTime = t; }
    void setTimeStep(double ts) { timeStep = ts; }
    void setSaveStats(bool s) { saveStates = s; }

private:
   double getMass();
   double getForceX();
   double getForceY();
   double getForceZ();

   double getTorqueP(); // yaw
   double getTorqueQ(); // pitch
   double getTorqueR(); // roll

   double getIyaw()   { return 1.0; }
   double getIpitch() { return 1.0; }
   double getIroll()  { return 1.0; }

   std::unique_ptr<sim::RK4Solver<Vector3>> linearIntegrator;
//   std::unique_ptr<sim::RK4Solver<Quaternion>> orientationIntegrator;

   std::shared_ptr<Rocket> rocket;

   StateData currentState;
   StateData nextState;

   Vector3 currentGravity{0.0, 0.0, 0.0};
   Vector3 currentWindSpeed{0.0, 0.0, 0.0};

   bool saveStates{true};
   double currentTime{0.0};
   double timeStep{0.01};
   std::vector<std::pair<double, StateData>> states;

   Vector3 getCurrentGravity();
};

} // namespace sim

#endif // SIM_PROPAGATOR_H
