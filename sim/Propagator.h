#ifndef SIM_PROPAGATOR_H
#define SIM_PROPAGATOR_H

#include "sim/DESolver.h"

#include <memory>
#include <vector>

// Forward declare
class Rocket;
class QtRocket;

namespace sim
{

class Propagator
{
public:
    Propagator(Rocket* r);
    ~Propagator();

    void setInitialState(const std::vector<double>& initialState)
    {
       currentState.resize(initialState.size());
       for(std::size_t i = 0; i < initialState.size(); ++i)
       {
           currentState[i] = initialState[i];
       }

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

    const std::vector<std::pair<double, std::vector<double>>>& getStates() const { return states; }

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

//private:

   std::unique_ptr<sim::DESolver> integrator;

   Rocket* rocket;
   QtRocket* qtrocket;

   std::vector<double> currentState{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   std::vector<double> tempRes{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   bool saveStates{true};
   double currentTime{0.0};
   double timeStep{0.01};
   std::vector<std::pair<double, std::vector<double>>> states;
};

} // namespace sim

#endif // SIM_PROPAGATOR_H
