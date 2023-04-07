#ifndef SIM_CONSTANTGRAVITYMODEL_H
#define SIM_CONSTANTGRAVITYMODEL_H

#include "GravityModel.h"

namespace sim {

class ConstantGravityModel : public GravityModel
{
public:
    ConstantGravityModel();

    virtual ~ConstantGravityModel() {}

    std::tuple<double, double, double> getAccel(double, double, double) override { return std::make_tuple(0.0, 0.0, 9.8); }
};

} // namespace sim

#endif // SIM_CONSTANTGRAVITYMODEL_H
