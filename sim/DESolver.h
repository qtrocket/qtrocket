#ifndef SIM_DESOLVER_H
#define SIM_DESOLVER_H

namespace sim
{

class DESolver
{
public:
   DESolver() {}
   virtual ~DESolver() {}

   virtual double step(double curVal, double t, double dt) = 0;
};

} // namespace sim

#endif // SIM_DESOLVER_H
