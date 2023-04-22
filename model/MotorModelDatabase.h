#ifndef MOTORMODELDATABASE_H
#define MOTORMODELDATABASE_H

/// \cond
// C headers
// C++ headers
#include <vector>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"

/**
 * @brief The MotorModelDatabase class provides a storage and search mechanism for
 *        MotorModels
 */
class MotorModelDatabase
{
public:
    /**
     * @brief MotorModelDatabase constructor
     */
    MotorModelDatabase();

    /**
     * @brief MotorModelDatabase destructor is defaulted
     */
    ~MotorModelDatabase() = default;

    /**
     * @brief findMotorsByManufacturer returns a vector of MotorModel from a given
     *        manufacturer
     * @param manufacturer The manufacturer to search for
     * @return vector of MotorModels from a given manufacturer
     */
    std::vector<model::MotorModel> findMotorsByManufacturer(const std::string& manufacturer);

    /**
     * @brief findMotersByImpulseClass returns a vector of MotorModels with a given
     *        impulse class
     * @param imClass Impulse class to search for
     * @return vector of MotorModels with a given Impulse class
     */
    std::vector<model::MotorModel> findMotersByImpulseClass(const std::string& imClass);

private:

    std::vector<model::MotorModel> motors;

};

#endif // MOTORMODELDATABASE_H
