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

#include "sim/ConstantAtmosphere.h"
#include "sim/USStandardAtmosphere.h"

namespace sim
{

/**
 * @brief Holds simulation environment information, such as the gravity model, atmosphere model,
 *        Geoid model
 * 
 */
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
            atmosphereModels[atmosphereModel].reset(new sim::ConstantAtmosphere);
        }
        else if(model == "US Standard 1976")
        {
            atmosphereModel = model;
            atmosphereModels[atmosphereModel].reset(new sim::USStandardAtmosphere);
        }
    }

    std::shared_ptr<sim::AtmosphericModel> getAtmosphericModel()
    {
        auto retVal = atmosphereModels[atmosphereModel];
        return retVal;
    }
    std::shared_ptr<sim::GravityModel> getGravityModel() { return gravityModels[gravityModel]; }

private:

    std::map<std::string, std::shared_ptr<sim::AtmosphericModel>> atmosphereModels{
        {"Constant Atmosphere", std::shared_ptr<sim::AtmosphericModel>()},
        {"US Standard 1976", std::shared_ptr<sim::AtmosphericModel>()}};

    std::map<std::string, std::shared_ptr<GravityModel>> gravityModels{
        {"Constant Gravity", std::shared_ptr<sim::GravityModel>()},
        {"Spherical Gravity", std::shared_ptr<sim::GravityModel>()}};

    std::string gravityModel{"Constant Gravity"}; /// Constant Gravity Model is the default
    std::string atmosphereModel{"Constant Atmosphere"}; /// Constant Atmosphere Model is the default
};

} // namespace sim


#endif // SIM_ENVIRONMENT_H
