/// \cond
// C headers
// C++ headers

// 3rd party headers
/// \endcond

// qtrocket headers
#include "Stage.h"

namespace model
{

Stage::Stage()
{}

Stage::~Stage()
{}

void Stage::setMotorModel(const model::MotorModel& motor)
{
   mm = motor;
}

} // namespace model
