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
   /**
    * Constructor takes a vector of pairs. The first item a timestamp,
    * the second the thrust in newtons.
   */
   ThrustCurve(std::vector<std::pair<double, double>>& tc);
   /**
    * Default constructor. Will create an empty thrustcurve, always returning 0.0
    * for all requested times.
   */
   ThrustCurve();
   ThrustCurve(const ThrustCurve&) = default;
   ThrustCurve(ThrustCurve&&) = default;
   ~ThrustCurve();

   ThrustCurve& operator=(const ThrustCurve& rhs) = default;

   ThrustCurve& operator=(ThrustCurve&& rhs) = default;

   /**
    * Assuming that the thrust is one dimensional. Seems reasonable, but just
    * documenting that for the record. For timesteps between known points the thrust
    * is interpolated linearly
    * @param t The time in seconds. For t > burntime or < 0, this will return 0.0
    * @return Thrust in Newtons
   */
   double getThrust(double t);

   void setIgnitionTime(double t);

   double getMaxTime() const { return maxTime; }

   /**
    * TODO: Get rid of this. This is for temporary testing
    */
   void setThrustCurveVector(const std::vector<std::pair<double, double>>& v);

   const std::vector<std::pair<double, double>> getThrustCurveData() const { return thrustCurve; }

private:
   std::vector<std::pair<double, double>> thrustCurve;
   double maxTime{0.0};
   double ignitionTime{0.0};
};

#endif // MODEL_THRUSTCURVE_H
