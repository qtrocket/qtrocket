#ifndef SIM_PROPAGATOR_H
#define SIM_PROPAGATOR_H

#include "sim/DESolver.h"

#include <memory>
#include <vector>

namespace sim
{

class Propagator
{
public:
    Propagator();
    ~Propagator();

    void setInitialState(std::vector<double>& initialState)
    {
       currentState = initialState;
    }

    const std::vector<double>& getCurrentState() const
    {
       return currentState;
    }

    void runUntilTerminate();

    void retainStates(bool s)
    {
       saveStates = s;
    }

    const std::vector<std::vector<double>>& getStates() const { return states; }

    void setTimeStep(double ts) { timeStep = ts; }

private:
    double getForceX() { return 0.0; }
    double getForceY() { return 0.0; }
    double getForceZ() { return 0.0; }

    double getTorqueP() { return 0.0; }
    double getTorqueQ() { return 0.0; }
    double getTorqueR() { return 0.0; }

   double getMass() { return 0.0; }

//private:

   std::unique_ptr<sim::DESolver> integrator;

   std::vector<double> currentState{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   std::vector<double> nextState{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   bool saveStates{true};
   double currentTime{0.0};
   double timeStep{0.01};
   std::vector<std::vector<double>> states;
};

} // namespace sim

#endif // SIM_PROPAGATOR_H
