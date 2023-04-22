#ifndef SIM_CONSTANTGRAVITYMODEL_H
#define SIM_CONSTANTGRAVITYMODEL_H

// qtrocket headers
#include "sim/GravityModel.h"
#include "utils/Triplet.h"

namespace sim {

class ConstantGravityModel : public GravityModel
{
public:
    ConstantGravityModel();

    virtual ~ConstantGravityModel() {}

    TripletD getAccel(double, double, double) override { return TripletD(0.0, 0.0, -9.8); }
};

} // namespace sim

#endif // SIM_CONSTANTGRAVITYMODEL_H
