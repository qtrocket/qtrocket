#ifndef SIM_WINDMODEL_H
#define SIM_WINDMODEL_H

#include <tuple>

namespace sim
{

class WindModel
{
public:
   WindModel();
   virtual ~WindModel();

   virtual std::tuple<double, double, double> getWindSpeed(double x, double y, double z);

};

} // namespace sim

#endif // SIM_WINDMODEL_H