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

namespace model
{

class Propagatable
{
public:
   Propagatable() {}
   virtual ~Propagatable() {}

   virtual double getDragCoefficient() = 0;
   virtual void setDragCoefficient(double d) = 0;

   virtual bool terminateCondition(const std::pair<double, StateData>&) = 0;

   virtual double getMass(double t) = 0;
   virtual double getThrust(double t) = 0;
private:

   sim::Aero aeroData;

   StateData state;
};

}

#endif // MODEL_PROPAGATABLE_H