#ifndef SIM_INTEGRATOR_H
#define SIM_INTEGRATOR_H
/// \cond
// C headers
// C++ headers
#include <algorithm>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <iterator>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "sim/DESolver.h"
#include "sim/RK4Solver.h"
#include "sim/RK45Solver.h"
#include "utils/Logger.h"
#include "utils/math/MathTypes.h"

namespace sim
{

/// Selects the DESolver backend (RK4 or RKF45) at runtime and forwards the ODE step to it.
class Integrator
{
public:
    Integrator()
    {
        // Seed the keys with null solvers (no brace-init: unique_ptr is move-only, but
        // initializer_list copies).
        integratorModels.emplace("Runge-Kutta 4th Order", nullptr);
        integratorModels.emplace("Runge-Kutta-Fehlberg", nullptr);
        setIntegratorModel("Runge-Kutta 4th Order");
    }

    ~Integrator() = default;
    Integrator(const Integrator&) = delete;
    Integrator(Integrator&&) = delete;
    Integrator& operator=(const Integrator&) = delete;
    Integrator& operator=(Integrator&&) = delete;

    std::vector<std::string> getAvailableIntegratorModels()
    {
        std::vector<std::string> retVal;
        retVal.reserve(integratorModels.size());
        std::transform(integratorModels.begin(), integratorModels.end(), std::back_inserter(retVal),
                  [](auto& i) { return i.first; });
        return retVal;
    }

    void setIntegratorModel(const std::string& model)
    {
        if(model == "Runge-Kutta 4th Order")
        {
            integratorModel = model;
            integratorModels[integratorModel] = std::make_unique<sim::RK4Solver<Vector3>>(odes);
        }
        else if(model == "Runge-Kutta-Fehlberg")
        {
            integratorModel = model;
            integratorModels[integratorModel] = std::make_unique<sim::RK45Solver<Vector3>>(odes);
        }
        else {
            // unknown name: logged no-op, keep the current valid model
            utils::Logger::getInstance()->error(
                "Integrator::setIntegratorModel: unknown model '" + model
                + "'; keeping the current model '" + integratorModel + "'.");
        }
    }

    void setIntegratorFunction(std::function<std::pair<Vector3, Vector3>(double, Vector3&, Vector3&)> func)
    {
        odes = std::move(func);
    }

    void setTimeStep(double dt)
    {
        integratorModels[integratorModel]->setTimeStep(dt);
    }

    StepResult<Vector3> step(double t, Vector3& state, Vector3& rate) { return integratorModels[integratorModel]->step(t, state, rate); }

private:

    std::map<std::string, std::unique_ptr<sim::DESolver<Vector3>>> integratorModels;

    std::string integratorModel{"Runge-Kutta 4th Order"}; /// RK4 Model is the default

    // This is the physics model
    std::function<std::pair<Vector3, Vector3>(double, Vector3&, Vector3&)> odes;
};

} // namespace sim


#endif // SIM_INTEGRATOR_H
