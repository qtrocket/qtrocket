#ifndef MODEL_RASPLOADER_H
#define MODEL_RASPLOADER_H

/// \cond
// C headers
// C++ headers
#include <string>
#include <vector>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"

namespace model {

class RASPLoader
{
public:
    explicit RASPLoader(const std::string& filename);

    std::vector<model::MotorModel>& getMotors() { return motors; }

    model::MotorModel getMotorModelByName(const std::string& name);

private:
    std::vector<model::MotorModel> motors;
};

} // namespace model

#endif // MODEL_RASPLOADER_H
