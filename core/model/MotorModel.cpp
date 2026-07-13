
/// \cond
// C headers
// C++ headers
#include <cctype>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"
#include "utils/math/Constants.h"
#include "utils/math/UtilityMathFunctions.h"
#include "utils/Logger.h"

namespace model
{

MotorModel::MotorModel()
{

}

MotorModel::~MotorModel()
{

}

double MotorModel::getMass(double simTime) const
{
    // empty mass + remaining propellant mass

    if(!ignitionOccurred)
    {
        return data.totalWeight;
    }
    else if(simTime - ignitionTime <= data.burnTime)
    {
        double thrustTime = simTime - ignitionTime;
        // Find the massCurve interval containing thrustTime; exact-hit returns directly, else interpolate.
        auto i = massCurve.cbegin();
        while(i->first <= thrustTime)
        {
            if(utils::math::floatingPointEqual(i->first, thrustTime))
            {
                return emptyMass + i->second;
            }
            else
            {
                i++;
            }
        }
        double tStart = std::prev(i)->first;
        double tEnd = i->first;
        double propMassStart = std::prev(i)->second;
        double propMassEnd   = i->second;
        double slope = (propMassEnd - propMassStart) / (tEnd - tStart);
        double currentMass = emptyMass + propMassStart + (thrustTime - tStart) * slope;
        utils::Logger::getInstance()->perf("simTime: " + std::to_string(simTime) + ": motor mass: " + std::to_string(currentMass));
        return currentMass;

    }
    // motor has burned out
    else
    {
        return emptyMass;
    }
}

double MotorModel::getThrust(double simTime)
{

    if(simTime > thrust.getMaxTime() + ignitionTime)
    {
        if(!burnOutOccurred)
        {
            utils::Logger::getInstance()->info("motor burnout occurred: " + std::to_string(simTime));
            burnOutOccurred = true;
        }
        return 0.0;
    }
    utils::Logger::getInstance()->perf("simTime: " + std::to_string(simTime) + ": thrust: " + std::to_string(thrust.getThrust(simTime)));
    return thrust.getThrust(simTime);
}

void MotorModel::setMetaData(const MetaData& md)
{
    data = md;
    computeMassCurve();
}

void MotorModel::moveMetaData(MetaData&& md)
{
    data = std::move(md);
    computeMassCurve();
}

std::string MotorModel::MetaData::deriveImpulseClass(const std::string& motorCode)
{
    auto first = motorCode.begin();
    while(first != motorCode.end() &&
            std::isspace(static_cast<unsigned char>(*first)) != 0)
    {
        ++first;
    }

    std::string code(first, motorCode.end());
    std::string upperCode;
    upperCode.reserve(code.size());
    for(char c : code)
        upperCode.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));

    if(upperCode.starts_with("1/4A"))
        return "1/4A";
    if(upperCode.starts_with("1/2A"))
        return "1/2A";

    for(char c : upperCode)
    {
        if(std::isalpha(static_cast<unsigned char>(c)) != 0)
            return std::string(1, c);
    }
    return "";
}

void MotorModel::computeMassCurve()
{
    emptyMass = data.totalWeight - data.propWeight;

    // Isp = total impulse / (g0 * propellant weight); propWeight is kg (every loader converts).
    isp = data.totalImpulse / (utils::math::Constants::g0 * data.propWeight);

    // Precompute the mass curve as a lookup table: keeps repeated getMass()/getThrust() calls within
    // one time step consistent, and speeds up the sim. 128 sample points (RASP files cap at 32).
    massCurve.reserve(128);
    double timeStep = data.burnTime / 127.0;
    double t = 0.0;
    double propMass{data.propWeight};
    for(std::size_t i = 0; i < 127; ++i)
    {
        massCurve.push_back(std::make_pair(t + i*timeStep, propMass));
        propMass -= thrust.getThrust(t + i*timeStep) * timeStep * data.propWeight / data.totalImpulse;
    }
    massCurve.push_back(std::make_pair(data.burnTime, 0.0));
}
} // namespace model
