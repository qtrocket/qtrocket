#ifndef SIM_STAGE_H
#define SIM_STAGE_H

/// \cond
// C headers
// C++ headers
#include <string>
#include <memory>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"
#include "model/Part.h"
#include "model/Propagatable.h"

namespace model
{

class Stage : public Propagatable
{
public:
   Stage();
   virtual ~Stage();

      /**
    * @brief setMotorModel
    * @param motor
    */
   void setMotorModel(const model::MotorModel& motor);

   /**
    * @brief Returns the current motor model.
    * @return The current motor model
    */
   model::MotorModel& getMotorModel() { return mm; }

   virtual double getMass(double t) override
   {
      if(topPart)
      {
         return topPart->getCompositeMass(t) + mm.getMass(t);
      }
      else
         return 0.0;
   }

   virtual double getDragCoefficient() override { return 1.0 ;}
   virtual void setDragCoefficient(double d) override {}

   virtual bool terminateCondition(const std::pair<double, StateData>& cond) override
   {
   // Terminate propagation when the z coordinate drops below zero
    if(cond.second.position[2] < 0)
        return true;
    else
        return false;
   }

   virtual double getThrust(double t) override { return mm.getThrust(t); }


private:
   std::string name;

   std::shared_ptr<Part> topPart;

   model::MotorModel mm;
   Vector3 motorModelPosition; // position of motor cg w.r.t. the stage c.g.
};

} // namespace model

#endif // SIM_STAGE_H