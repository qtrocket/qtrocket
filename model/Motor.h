#ifndef MODEL_MOTOR_H
#define MODEL_MOTOR_H

// For boost serialization. We're using boost::serialize to save
// and load Motor data to file
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <string>

#include "Thrustcurve.h"
#include "MotorCase.h"

class Motor
{
public:
   Motor();
   ~Motor();
private:
   // Needed for boost serialize
   friend class boost::serialization::access;
   template<class Archive>
   void serialize(Archive& ar, const unsigned int version);

   std::string manufacturer;
   std::string impulseClass; // 'A', 'B', '1/2A', 'M', etc
   std::string propType;
   bool sparky;

   // Thrust parameters
   double totalImpulse;
   double avgImpulse;
   int delay;
   double burnTime;
   double isp;
   MotorCase motorCase;
   Thrustcurve thrust;
   
   // Physical dimensions
   int diameter;
   int length;
   double totalWeight;
   double propWeight;

};

template<class Archive>
void Motor::serialize(Archive& ar, const unsigned int version)
{
   ar & manufacturer;
   ar & impulseClass;
   ar & propType;
   ar & sparky;
   ar & totalImpulse;
   ar & avgImpulse;
   ar & delay;
   ar & burnTime;
   ar & isp;
   ar & motorCase;
   ar & thrust;
   ar & diameter;
   ar & length;
   ar & totalWeight;
   ar & propWeight;
}

#endif // MODEL_MOTOR_H
