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
#include "sim/TrajectoryStatistics.h"
#include "utils/math/MathTypes.h"

// Forward declarations
namespace sim { class Environment; }

namespace model
{

class Propagatable
{
public:
   Propagatable();
   virtual ~Propagatable() {}

   virtual Vector3 getForces(double t, const Vector3& position, const Vector3& velocity, sim::Environment& environment) = 0;
   virtual Vector3 getTorques(double t) = 0;

   virtual double getMass(double t) = 0;
   virtual Matrix3 getCompositeInertiaTensor(double t) = 0;

   /// @brief Fill st.mass/cg/inertia with the body's composite mass properties at time t. Intended to be called by
   ///        the Propagator once per recorded step (before appendState), so the trajectory carries
   ///        mass/CG/I(t).
   virtual void writeMassProperties(double t, StateData& st) = 0;

   virtual bool terminateCondition(double t) = 0;

   void setCurrentState(const StateData& st) { currentState = st; }
   const StateData& getCurrentState() { return currentState; }

   const StateData& getInitialState() { return initialState; }
   void setInitialState(const StateData& init) { initialState = init; }

   void appendState(double t, const StateData& st) { states.emplace_back(t, st); }

   const std::vector<std::pair<double, StateData>>& getStates() { return states; }

   void clearStates() { states.clear(); }

   /// Running whole-trajectory statistics (max altitude/speed, time to apogee, total flight
   /// time). The Propagator updates these every step during runUntilTerminate -- independently
   /// of the state history above, so the summary and hang detection work even when state
   /// saving is off.
   void updateTrajectoryStatistics(double t, const StateData& st) { stats.update(t, st); }
   void resetTrajectoryStatistics() { stats.reset(); }
   const sim::TrajectoryStatistics& getTrajectoryStatistics() const { return stats; }

protected:

   sim::Aero aeroData;

   StateData initialState;
   StateData currentState;
   StateData nextState;

   std::vector<std::pair<double, StateData>> states;

   sim::TrajectoryStatistics stats;
};

}

#endif // MODEL_PROPAGATABLE_H
