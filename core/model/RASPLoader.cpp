/// \cond
// C headers
// C++ headers
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/RASPLoader.h"
#include "utils/Logger.h"

namespace
{

struct SourceLine
{
    std::size_t number{0};
    std::string text;
};

struct RaspHeader
{
    std::string code;
    double diameter{0.0};
    double length{0.0};
    std::string delays;
    double propWeight{0.0};
    double totalWeight{0.0};
    std::string manufacturer;
};

std::string trim(const std::string& s)
{
    const auto first = std::find_if_not(s.begin(), s.end(),
                                                    [](unsigned char c)
                                                    {
                                                        return std::isspace(c) != 0;
                                                    });
    const auto last = std::find_if_not(s.rbegin(), s.rend(),
                                                   [](unsigned char c)
                                                   {
                                                       return std::isspace(c) != 0;
                                                   }).base();
    if(first >= last)
        return "";
    return std::string(first, last);
}

bool isIgnoredLine(const std::string& line)
{
    const std::string t = trim(line);
    return t.empty() || t[0] == ';';
}

std::runtime_error parseError(const SourceLine& line, const std::string& message)
{
    return std::runtime_error("RASP parse error on line " + std::to_string(line.number) +
                                       ": " + message);
}

double parseDouble(const std::string& token, const SourceLine& line, const std::string& field)
{
    try
    {
        std::size_t parsed = 0;
        const double value = std::stod(token, &parsed);
        if(parsed != token.size() || !std::isfinite(value))
            throw parseError(line, field + " must be a finite number");
        return value;
    }
    catch(const std::invalid_argument&)
    {
        throw parseError(line, field + " must be a finite number");
    }
    catch(const std::out_of_range&)
    {
        throw parseError(line, field + " is out of range");
    }
}

int parseDelay(const std::string& token, const SourceLine& line)
{
    std::string upper;
    upper.reserve(token.size());
    for(char c : token)
        upper.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));

    if(upper == "P" || upper == "NONE")
        return 1000;

    try
    {
        std::size_t parsed = 0;
        const int delay = std::stoi(token, &parsed);
        if(parsed != token.size() || delay < 0)
            throw parseError(line, "delay values must be non-negative integers or P");
        return delay;
    }
    catch(const std::invalid_argument&)
    {
        throw parseError(line, "delay values must be non-negative integers or P");
    }
    catch(const std::out_of_range&)
    {
        throw parseError(line, "delay value is out of range");
    }
}

std::vector<int> parseDelays(const std::string& delays, const SourceLine& line)
{
    std::vector<int> parsed;
    std::size_t start = 0;
    while(start <= delays.size())
    {
        const std::size_t dash = delays.find('-', start);
        const std::string token = delays.substr(start, dash - start);
        if(token.empty())
            throw parseError(line, "delay list contains an empty value");
        parsed.push_back(parseDelay(token, line));
        if(dash == std::string::npos)
            break;
        start = dash + 1;
    }
    return parsed;
}

RaspHeader parseHeader(const SourceLine& line)
{
    std::istringstream iss(line.text);
    std::vector<std::string> tokens;
    std::string token;
    while(iss >> token)
        tokens.push_back(token);

    if(tokens.size() != 7)
        throw parseError(line, "header must contain 7 whitespace-separated fields");

    RaspHeader h;
    h.code = tokens[0];
    h.diameter = parseDouble(tokens[1], line, "diameter");
    h.length = parseDouble(tokens[2], line, "length");
    h.delays = tokens[3];
    h.propWeight = parseDouble(tokens[4], line, "propellant weight");
    h.totalWeight = parseDouble(tokens[5], line, "initial weight");
    h.manufacturer = tokens[6];

    if(h.diameter <= 0.0)
        throw parseError(line, "diameter must be positive");
    if(h.length <= 0.0)
        throw parseError(line, "length must be positive");
    if(h.propWeight < 0.0)
        throw parseError(line, "propellant weight must be non-negative");
    if(h.totalWeight <= 0.0)
        throw parseError(line, "initial weight must be positive");
    if(h.totalWeight < h.propWeight)
        throw parseError(line, "initial weight must be at least propellant weight");

    return h;
}

std::optional<std::pair<double, double>> tryParseSample(const SourceLine& line)
{
    std::istringstream iss(line.text);
    std::string timeToken;
    std::string thrustToken;
    std::string extra;
    if(!(iss >> timeToken >> thrustToken) || (iss >> extra))
        return std::nullopt;

    return std::make_pair(parseDouble(timeToken, line, "sample time"),
                                 parseDouble(thrustToken, line, "sample thrust"));
}

bool isZero(double value)
{
    return std::abs(value) <= std::numeric_limits<double>::epsilon();
}

double integrateImpulse(const std::vector<std::pair<double, double>>& samples)
{
    double total = 0.0;
    for(std::size_t i = 1; i < samples.size(); ++i)
    {
        const double dt = samples[i].first - samples[i - 1].first;
        total += 0.5 * (samples[i - 1].second + samples[i].second) * dt;
    }
    return total;
}

model::MotorModel buildMotor(const RaspHeader& header,
                                       const SourceLine& headerLine,
                                       std::vector<std::pair<double, double>> thrustData)
{
    if(thrustData.empty())
        throw parseError(headerLine, "motor has no thrust samples");

    if(thrustData.front().first > 0.0)
        thrustData.insert(thrustData.begin(), {0.0, 0.0});

    const double burnTime = thrustData.back().first;
    const double totalImpulse = integrateImpulse(thrustData);
    if(burnTime <= 0.0)
        throw parseError(headerLine, "burn time must be positive");
    if(totalImpulse <= 0.0)
        throw parseError(headerLine, "total impulse must be positive");

    double maxThrust = 0.0;
    for(const auto& [time, thrust] : thrustData)
    {
        (void)time;
        maxThrust = std::max(maxThrust, thrust);
    }

    model::MotorModel::MetaData md;
    md.availability = model::MotorModel::MotorAvailability(model::MotorModel::AVAILABILITY::REGULAR);
    md.avgThrust = totalImpulse / burnTime;
    md.burnTime = burnTime;
    md.certOrg = model::MotorModel::CertOrg(model::MotorModel::CERTORG::UNK);
    md.commonName = header.code;
    md.delays = parseDelays(header.delays, headerLine);
    md.designation = header.code;
    md.diameter = header.diameter;
    md.impulseClass = model::MotorModel::MetaData::deriveImpulseClass(header.code);
    md.length = header.length;
    md.manufacturer = model::MotorModel::MotorManufacturer::toEnum(header.manufacturer);
    md.maxThrust = maxThrust;
    md.propWeight = header.propWeight;
    md.totalImpulse = totalImpulse;
    md.totalWeight = header.totalWeight;
    md.type = model::MotorModel::MotorType(model::MotorModel::MOTORTYPE::SU);

    model::MotorModel motor;
    motor.addThrustCurve(ThrustCurve(thrustData));
    motor.moveMetaData(std::move(md));
    return motor;
}

} // namespace

namespace model
{

RASPLoader::RASPLoader(const std::string& filename)
    : motors()
{
    std::ifstream file(filename);
    if(!file.is_open())
        throw std::runtime_error("Unable to open RASP file: " + filename);

    std::vector<SourceLine> lines;
    std::string text;
    for(std::size_t number = 1; std::getline(file, text); ++number)
        lines.push_back(SourceLine{number, trim(text)});

    std::size_t i = 0;
    while(i < lines.size())
    {
        while(i < lines.size() && isIgnoredLine(lines[i].text))
            ++i;
        if(i >= lines.size())
            break;

        const SourceLine& headerLine = lines[i];
        const RaspHeader header = parseHeader(headerLine);
        ++i;

        std::vector<std::pair<double, double>> thrustData;
        double previousTime = 0.0;
        bool sawFinalZero = false;

        while(i < lines.size())
        {
            if(isIgnoredLine(lines[i].text))
            {
                ++i;
                continue;
            }

            if(sawFinalZero)
            {
                if(tryParseSample(lines[i]).has_value())
                    throw parseError(lines[i], "zero-thrust sample must be the final data point");
                break;
            }

            const auto sample = tryParseSample(lines[i]);
            if(!sample.has_value())
                throw parseError(lines[i], "sample must contain exactly time and thrust");

            const auto [time, thrust] = *sample;
            if(time < 0.0)
                throw parseError(lines[i], "sample time must be non-negative");
            if(!thrustData.empty() && time <= previousTime)
                throw parseError(lines[i], "sample times must be strictly increasing");
            if(thrust < 0.0)
                throw parseError(lines[i], "sample thrust must be non-negative");

            thrustData.push_back(*sample);
            previousTime = time;
            sawFinalZero = isZero(thrust);
            ++i;
        }

        if(!sawFinalZero)
            throw parseError(headerLine, "motor thrust data must end with a zero-thrust sample");

        motors.emplace_back(buildMotor(header, headerLine, std::move(thrustData)));
    }
}

model::MotorModel RASPLoader::getMotorModelByName(const std::string& name)
{
    auto mm = std::find_if(motors.begin(), motors.end(),
                                   [&name](const auto& i) { return name == i.data.commonName; });
    if(mm == motors.end())
    {
        utils::Logger::getInstance()->error("Unable to locate " + name + " in RASP database");
        return model::MotorModel();
    }
    return *mm;
}

} // namespace model
