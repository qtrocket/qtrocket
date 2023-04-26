#ifndef SIMULATIONOPTIONS_H
#define SIMULATIONOPTIONS_H

/// \cond
// C headers
// C++ headers
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "sim/GravityModel.h"
#include "sim/SphericalGravityModel.h"
#include "sim/ConstantGravityModel.h"

#include "sim/AtmosphericModel.h"
#include "sim/ConstantAtmosphere.h"
#include "sim/USStandardAtmosphere.h"

namespace sim
{

/**
 * @brief The SimulationOptions class holds the available simulation options and environmental models
 */
class SimulationOptions
{
public:
    SimulationOptions()
    {
        //setTimeStep(0.01);
        //setGravityModel("Constant Gravity");
        //setAtmosphereModel("Constant Atmosphere");
    }
    ~SimulationOptions() = default;
    SimulationOptions(const SimulationOptions&) = delete;
    SimulationOptions(SimulationOptions&&) = delete;
    SimulationOptions& operator=(const SimulationOptions&) = delete;
    SimulationOptions& operator=(SimulationOptions&&) = delete;

    std::vector<std::string> getAvailableGravityModels()
    {
        std::vector<std::string> retVal(gravityModels.size());
        std::transform(gravityModels.begin(), gravityModels.end(), std::back_inserter(retVal),
                  [](auto& i) { return i.first; });
        return retVal;
    }

    std::vector<std::string> getAvailableAtmosphereModels()
    {
        std::vector<std::string> retVal(atmosphereModels.size());
        std::transform(atmosphereModels.begin(), atmosphereModels.end(), std::back_inserter(retVal),
                  [](auto& i) { return i.first; });
        return retVal;
    }

    void setTimeStep(double t) { timeStep = t; }
    void setGravityModel(const std::string& model)
    {
        if(model == "Constant Gravity")
        {
            gravityModel = model;
            gravityModels[gravityModel].reset(new sim::ConstantGravityModel);
        }
        else if(model == "Spherical Gravity")
        {
            gravityModel = model;
            gravityModels[gravityModel].reset(new sim::SphericalGravityModel);
        }
    }

    void setAtmosphereModel(const std::string& model)
    {
        if(model == "Constant Atmosphere")
        {
            atmosphereModel = model;
            atmosphereModels[gravityModel].reset(new sim::ConstantAtmosphere);
        }
        else if(model == "US Standard 1976")
        {
            atmosphereModel = model;
            atmosphereModels[gravityModel].reset(new sim::USStandardAtmosphere);
        }
    }

    std::shared_ptr<sim::AtmosphericModel> getAtmosphericModel() { return atmosphereModels[atmosphereModel]; }
    std::shared_ptr<sim::GravityModel> getGravityModel() { return gravityModels[gravityModel]; }
    double getTimeStep() { return timeStep; }

private:

    std::map<std::string, std::shared_ptr<sim::AtmosphericModel>> atmosphereModels{
        {"Constant Atmosphere", std::shared_ptr<sim::AtmosphericModel>()},
        {"US Standard 1976", std::shared_ptr<sim::AtmosphericModel>()}};

    std::map<std::string, std::shared_ptr<GravityModel>> gravityModels{
        {"Constant Gravity", std::shared_ptr<sim::GravityModel>()},
        {"Spherical Gravity", std::shared_ptr<sim::GravityModel>()}};

    double timeStep{0.01};

    std::string gravityModel{"Constant Gravity"}; /// Constant Gravity Model is the default
    std::string atmosphereModel{"Constant Atmosphere"}; /// Constant Atmosphere Model is the default
};

}

#endif // SIMULATIONOPTIONS_H
