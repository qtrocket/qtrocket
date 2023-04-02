#ifndef ROCKET_H
#define ROCKET_H

#include "utils/math/Vector3.h"
#include "utils/math/Quaternion.h"

#include "sim/Propagator.h"

#include <utility> // std::move
#include <memory>

class Rocket
{
public:
   Rocket();

   void setPosition(const math::Vector3& pos) { *position = pos; }
   void setPosition(math::Vector3&& pos) { *position = std::move(pos); }

   void setVelocity(const math::Vector3& vel) { *velocity = vel; }
   void setVelocity(math::Vector3&& vel) { *velocity = std::move(vel); }

   void setOrientation(const math::Quaternion& ori) { *orientation = ori; }
   void setOrientation(math::Quaternion&& ori) { *orientation = std::move(ori); }

   void setOrientationRate(const math::Quaternion& ori) { *orientationRate = ori; }
   void setOrientationRate(math::Quaternion&& ori) { *orientationRate = std::move(ori); }

   const math::Vector3& getPosition() const { return *position; }
   const math::Vector3& getVelocity() const { return *velocity; }
   const math::Quaternion& getOrientation() const { return *orientation; }
   const math::Quaternion& getOrientationRate() const { return *orientation; }
private:

   std::shared_ptr<math::Vector3> position;
   std::shared_ptr<math::Vector3> velocity;
   std::shared_ptr<math::Quaternion> orientation;
   std::shared_ptr<math::Quaternion> orientationRate;

   sim::Propagator propagator;
};

#endif // ROCKET_H
