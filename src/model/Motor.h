#ifndef MODEL_MOTOR_H
#define MODEL_MOTOR_H

#include <string>

#include "Thrustcurve.h"
#include "MotorCase.h"

class Motor
{
public:
   Motor();
   ~Motor();
private:

   std::string manufacturer;
   char impulseClass;
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
#endif // MODEL_MOTOR_H