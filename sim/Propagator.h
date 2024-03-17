#ifndef SIM_PROPAGATOR_H
#define SIM_PROPAGATOR_H

/// \cond
// C headers
// C++ headers
#include <memory>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "sim/RK4Solver.h"
#include "utils/math/MathTypes.h"
#include "sim/StateData.h"
#include "model/Propagatable.h"


// Forward declare
namespace model
{
class Rocket;
}
class QtRocket;

namespace sim
{

class Propagator
{
public:
    Propagator(std::shared_ptr<model::Propagatable> o);
    ~Propagator();

    void setInitialState(const StateData& initialState)
    {
        object->setInitialState(initialState);
    }

    const StateData& getCurrentState() const
    {
        return object->getCurrentState();
    }

    void runUntilTerminate();

    void retainStates(bool s)
    {
       saveStates = s;
    }

    void setCurrentTime(double t) { currentTime = t; }
    void setTimeStep(double ts) { timeStep = ts; }
    void setSaveStats(bool s) { saveStates = s; }

private:

   std::unique_ptr<sim::RK4Solver<Vector3>> linearIntegrator;
//   std::unique_ptr<sim::RK4Solver<Quaternion>> orientationIntegrator;

   std::shared_ptr<model::Propagatable> object;

   bool saveStates{true};
   double currentTime{0.0};
   double timeStep{0.01};

};

} // namespace sim

#endif // SIM_PROPAGATOR_H
