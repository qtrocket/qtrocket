#ifndef MODEL_THRUSTCURVE_H
#define MODEL_THRUSTCURVE_H

#include <vector>
#include <tuple>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

// No namespace. Maybe should be, but this is a model rocket program
// so model is sort of implied? Or I'm just making excuses for being lazy

class Thrustcurve
{
public:
   /**
    * Constructor takes a vector of pairs. The first item a timestamp,
    * the second the thrust in newtons.
   */
   Thrustcurve(std::vector<std::pair<double, double>>& tc);
   /**
    * Default constructor. Will create an empty thrustcurve, always returning 0.0
    * for all requested times.
   */
   Thrustcurve();
   ~Thrustcurve();

   /**
    * Assuming that the thrust is one dimensional. Seems reasonable, but just
    * documenting that for the record. For timesteps between known points the thrust
    * is interpolated linearly
    * @param t The time in seconds. For t > burntime or < 0, this will return 0.0
    * @return Thrust in Newtons
   */
   double getThrust(double t);

private:
   // We're using boost::serialize for data storage and retrieval
   friend class boost::serialization::access;
   template<class Archive>
   void serialize(Archive& ar, const unsigned int version);

   std::vector<std::pair<double, double>> thrustCurve;
   double maxTime;
};

template<class Archive>
void Thrustcurve::serialize(Archive& ar, const unsigned int version)
{
   ar & maxTime;
   ar & thrustCurve;
}

#endif // MODEL_THRUSTCURVE_H
