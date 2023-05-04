#ifndef UTILS_MOTORMODELDATABASE_H
#define UTILS_MOTORMODELDATABASE_H

/// \cond
// C headers
// C++ headers
#include <vector>
#include <string>
#include <map>
#include <optional>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"


namespace utils
{
/**
 * @brief MotorModelDatabase is a simple storage, search, and retrieval mechanism for Model Rocket
 *        motors.
 * 
 */
class MotorModelDatabase
{
public:
   MotorModelDatabase();
   ~MotorModelDatabase();

   // No copies
   MotorModelDatabase(const MotorModelDatabase&) = delete;
   MotorModelDatabase(MotorModelDatabase&&) = delete;
   MotorModelDatabase& operator=(const MotorModelDatabase&) = delete;
   MotorModelDatabase& operator=(MotorModelDatabase&&) = delete;


   /**
    * @brief Adds a single MotorModel to the database. If that MotorModel already exists, it is
    *        replaced.
    * 
    * @param model MotorModel to add
    * 
    */
   void addMotorModel(const model::MotorModel& m);

   /**
    * @brief Adds multiple motor models at once. Any duplicates already in the datbase are replaced.
    * 
    * @param model MotorModels to add
    * 
    */
   void addMotorModels(const std::vector<model::MotorModel>& models);

   /**
    * @brief Get the Motor Model by Common Name
    * 
    * @param name Motor Common name
    * @return std::optional<model::MotorModel> 
    */
   std::optional<model::MotorModel> getMotorModel(const std::string& name);

   void saveMotorDatabase(const std::string& filename);
   void loadMotorDatabase(const std::string& filename);
private:

   // The "database" is really just a map. :)
   /// motorModelMap is keyed off of the motor commonName
   std::map<std::string, model::MotorModel> motorModelMap;
   


};

} // namespace utils

#endif // UTILS_MOTORMODELDATABASE_H
