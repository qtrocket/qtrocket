#ifndef MOTORMODELDATABASE_H
#define MOTORMODELDATABASE_H

#include <vector>
#include "model/MotorModel.h"

class MotorModelDatabase
{
public:
    MotorModelDatabase();

    std::vector<MotorModel> findMotorsByManufacturer(const std::string& manu);

    std::vector<MotorModel> findMotersByImpulseClass(const std::string& imClass);

    MotorModel getMotorByName(const std::string& name);

    std::vector<std::pair<double, double>> getThrustCurveByName(const std::string& motorName);

private:

    std::vector<MotorModel> motors;

};

#endif // MOTORMODELDATABASE_H
