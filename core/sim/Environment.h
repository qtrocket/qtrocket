#ifndef SIM_ENVIRONMENT_H
#define SIM_ENVIRONMENT_H
/// \cond
// C headers
// C++ headers
#include <algorithm>
#include <map>
#include <memory>
#include <vector>
#include <iterator>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "sim/ConstantGravityModel.h"
#include "sim/SphericalGravityModel.h"

#include "sim/SphericalGeoidModel.h"

#include "sim/ConstantAtmosphere.h"
#include "sim/USStandardAtmosphere.h"
#include "sim/VacuumAtmosphere.h"

namespace sim
{

/// Holds the pluggable physics models for a simulation: gravity, atmosphere, and geoid.
class Environment
{
public:
    Environment()
    {
        setGravityModel("Constant Gravity");
        setAtmosphereModel("Constant Atmosphere");
    }
    ~Environment() = default;
    Environment(const Environment&) = delete;
    Environment(Environment&&) = delete;
    Environment& operator=(const Environment&) = delete;
    Environment& operator=(Environment&&) = delete;

    std::vector<std::string> getAvailableGravityModels()
    {
        std::vector<std::string> retVal;
        retVal.reserve(gravityModels.size());
        std::transform(gravityModels.begin(), gravityModels.end(), std::back_inserter(retVal),
                  [](auto& i) { return i.first; });
        return retVal;
    }

    std::vector<std::string> getAvailableAtmosphereModels()
    {
        std::vector<std::string> retVal;
        retVal.reserve(atmosphereModels.size());
        std::transform(atmosphereModels.begin(), atmosphereModels.end(), std::back_inserter(retVal),
                  [](auto& i) { return i.first; });
        return retVal;
    }

    void setGravityModel(const std::string& model)
    {
        if(model == "Constant Gravity")
        {
            gravityModel = model;
            gravityModels[gravityModel] = std::make_shared<sim::ConstantGravityModel>();
        }
        else if(model == "Spherical Gravity")
        {
            gravityModel = model;
            gravityModels[gravityModel] = std::make_shared<sim::SphericalGravityModel>(geoidModel);
        }
    }

    void setAtmosphereModel(const std::string& model)
    {
        if(model == "Constant Atmosphere")
        {
            atmosphereModel = model;
            atmosphereModels[atmosphereModel] = std::make_shared<sim::ConstantAtmosphere>();
        }
        else if(model == "US Standard 1976")
        {
            atmosphereModel = model;
            atmosphereModels[atmosphereModel] = std::make_shared<sim::USStandardAtmosphere>();
        }
        else if(model == "Vacuum")
        {
            atmosphereModel = model;
            atmosphereModels[atmosphereModel] = std::make_shared<sim::VacuumAtmosphere>();
        }
    }

    std::shared_ptr<sim::AtmosphericModel> getAtmosphericModel()
    {
        auto retVal = atmosphereModels[atmosphereModel];
        return retVal;
    }
    std::shared_ptr<sim::GravityModel> getGravityModel() { return gravityModels[gravityModel]; }
    std::shared_ptr<sim::GeoidModel> getGeoidModel() { return geoidModel; }

private:

    std::map<std::string, std::shared_ptr<sim::AtmosphericModel>> atmosphereModels{
        {"Constant Atmosphere", std::shared_ptr<sim::AtmosphericModel>()},
        {"US Standard 1976", std::shared_ptr<sim::AtmosphericModel>()},
        {"Vacuum", std::shared_ptr<sim::AtmosphericModel>()}};

    std::map<std::string, std::shared_ptr<GravityModel>> gravityModels{
        {"Constant Gravity", std::shared_ptr<sim::GravityModel>()},
        {"Spherical Gravity", std::shared_ptr<sim::GravityModel>()}};

    std::string gravityModel{"Constant Gravity"}; /// Constant Gravity Model is the default
    std::string atmosphereModel{"Constant Atmosphere"}; /// Constant Atmosphere Model is the default

    /// Supplies the launch-site ground radius the Spherical Gravity model uses to map the local
    /// launch frame to a geocentric distance. Only one geoid today; a registry can follow if needed.
    std::shared_ptr<sim::GeoidModel> geoidModel{std::make_shared<sim::SphericalGeoidModel>()};
};

} // namespace sim


#endif // SIM_ENVIRONMENT_H
