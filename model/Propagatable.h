#ifndef MODEL_PROPAGATABLE_H
#define MODEL_PROPAGATABLE_H

/// \cond
// C headers
// C++ headers
#include <utility>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "sim/Aero.h"
#include "sim/StateData.h"
#include "utils/math/MathTypes.h"

namespace model
{

class Propagatable
{
public:
   Propagatable() {}
   virtual ~Propagatable() {}

   virtual Vector3 getForces(double t) = 0;
   virtual Vector3 getTorques(double t) = 0;

   virtual double getMass(double t) = 0;
   virtual Matrix3 getInertiaTensor(double t) = 0;

   virtual bool terminateCondition(double t) = 0;

   void setCurrentState(const StateData& st) { currentState = st; }
   const StateData& getCurrentState() { return currentState; }

   const StateData& getInitialState() { return initialState; }
   void setInitialState(const StateData& init) { initialState = init; }

   void appendState(double t, const StateData& st) { states.emplace_back(t, st); }

   const std::vector<std::pair<double, StateData>>& getStates() { return states; }

   void clearStates() { states.clear(); }

protected:

   sim::Aero aeroData;

   StateData initialState;
   StateData currentState;
   StateData nextState;

   std::vector<std::pair<double, StateData>> states;
};

}

#endif // MODEL_PROPAGATABLE_H
