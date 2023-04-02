#ifndef SIM_PROPAGATOR_H
#define SIM_PROPAGATOR_H

#include "sim/DESolver.h"

#include <memory>

namespace sim
{

class Propagator
{
public:
    Propagator();

private:

   std::unique_ptr<sim::DESolver> solver;
};

} // namespace sim

#endif // SIM_PROPAGATOR_H
