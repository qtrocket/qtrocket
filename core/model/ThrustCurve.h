#ifndef MODEL_THRUSTCURVE_H
#define MODEL_THRUSTCURVE_H

/// \cond
// C headers
// C++ headers
#include <vector>

// 3rd party headers
/// \endcond

class ThrustCurve
{
public:
    /// @brief Build from (time seconds, thrust Newtons) samples.
    ThrustCurve(std::vector<std::pair<double, double>>& tc);
    /// @brief Empty curve: getThrust() returns 0 for all times.
    ThrustCurve();
    ThrustCurve(const ThrustCurve&) = default;
    ThrustCurve(ThrustCurve&&) = default;
    ~ThrustCurve();

    ThrustCurve& operator=(const ThrustCurve& rhs) = default;

    ThrustCurve& operator=(ThrustCurve&& rhs) = default;

    /// @brief Thrust (Newtons) at time @p t (seconds), linearly interpolated between samples. 0 for
    ///        t < 0 or t > burn time.
    double getThrust(double t);

    void setIgnitionTime(double t);

    double getMaxTime() const { return maxTime; }

    const std::vector<std::pair<double, double>> getThrustCurveData() const { return thrustCurve; }

private:
    std::vector<std::pair<double, double>> thrustCurve;
    double maxTime{0.0};
    double ignitionTime{0.0};
};

#endif // MODEL_THRUSTCURVE_H
