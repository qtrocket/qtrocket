#ifndef MODEL_THRUSTCURVE_H
#define MODEL_THRUSTCURVE_H

/// \cond
// C headers
// C++ headers
#include <vector>

// 3rd party headers
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
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

   ThrustCurve& operator=(const ThrustCurve& rhs)
   {
      if(this != &rhs)
      {
         thrustCurve = rhs.thrustCurve;
         maxTime = rhs.maxTime;
         ignitionTime = rhs.ignitionTime;
      }
      return *this;
   }

   ThrustCurve& operator=(ThrustCurve&& rhs)
   {
      thrustCurve = std::move(rhs.thrustCurve);
      maxTime = std::move(rhs.maxTime);
      ignitionTime = std::move(rhs.ignitionTime);
      return *this;
   }

   /**
    * Assuming that the thrust is one dimensional. Seems reasonable, but just
    * documenting that for the record. For timesteps between known points the thrust
    * is interpolated linearly
    * @param t The time in seconds. For t > burntime or < 0, this will return 0.0
    * @return Thrust in Newtons
   */
   double getThrust(double t);

   void setIgnitionTime(double t);

   /**
 * TODO: Get rid of this. This is for temporary testing
 */
   void setThrustCurveVector(const std::vector<std::pair<double, double>>& v);


private:
   // We're using boost::serialize for data storage and retrieval
   friend class boost::serialization::access;
   template<class Archive>
   void serialize(Archive& ar, const unsigned int version);

   std::vector<std::pair<double, double>> thrustCurve;
   double maxTime{0.0};
   double ignitionTime{0.0};
};

template<class Archive>
void ThrustCurve::serialize(Archive& ar, const unsigned int version)
{
   ar & maxTime;
   ar & thrustCurve;
}

#endif // MODEL_THRUSTCURVE_H