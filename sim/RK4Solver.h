#ifndef SIM_RK4SOLVER_H
#define SIM_RK4SOLVER_H

#include <functional>
#include <vector>
#include <limits>
#include <cmath>

#include "utils/Logger.h"
#include "sim/DESolver.h"

namespace sim {

template<typename... Ts>
class RK4Solver : public DESolver
{
public:

   RK4Solver(Ts... funcs)
   {
      (odes.push_back(funcs), ...);

   }
   virtual ~RK4Solver() {}

   void setTimeStep(double inTs) override { dt = inTs;  halfDT = dt / 2.0; }

   void step(double t, double* curVal, double* res) override
   {
      if(dt == std::numeric_limits<double>::quiet_NaN())
      {
         utils::Logger::getInstance()->error("Calling RK4Solver without setting dt first is an error");
         res[0] = std::numeric_limits<double>::quiet_NaN();
      }

      for(size_t i = 0; i < len; ++i)
      {
         k1[i] = odes[i](t, curVal);
      }
      // compute k2 values. This involves stepping the current values forward a half-step
      // based on k1, so we do the stepping first
      for(size_t i = 0; i < len; ++i)
      {
         temp[i] = curVal[i] + k1[i]*dt / 2.0;
      }
      for(size_t i = 0; i < len; ++i)
      {
         k2[i] = odes[i](t + halfDT, temp);
      }
      // repeat for k3
      for(size_t i = 0; i < len; ++i)
      {
         temp[i] = curVal[i] + k2[i]*dt / 2.0;
      }
      for(size_t i = 0; i < len; ++i)
      {
         k3[i] = odes[i](t + halfDT, temp);
      }

      // now k4
      for(size_t i = 0; i < len; ++i)
      {
         temp[i] = curVal[i] + k3[i]*dt;
      }
      for(size_t i = 0; i < len; ++i)
      {
         k4[i] = odes[i](t + dt, temp);
      }

      // now compute the result
      for(size_t i = 0; i < len; ++i)
      {
         res[i] = curVal[i] + (dt / 6.0)*(k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
      }

   }

private:
   std::vector<std::function<double(double, double*)>> odes;

   static constexpr size_t len = sizeof...(Ts);
   double k1[len];
   double k2[len];
   double k3[len];
   double k4[len];

   double temp[len];

   double dt = std::numeric_limits<double>::quiet_NaN();
   double halfDT = 0.0;

};

} // namespace sim

#endif // SIM_RK4SOLVER_H
