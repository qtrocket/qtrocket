// class header
#include "utils/MotorModelDatabase.h"

/// \cond
// C headers
// C++ headers
// 3rd party headers
/// \endcond

// qtrocket project headers
#include "QtRocket.h"

namespace utils
{

MotorModelDatabase::MotorModelDatabase()
   : motorModelMap()
{
}

MotorModelDatabase::~MotorModelDatabase()
{
}

void MotorModelDatabase::addMotorModel(const model::MotorModel& m)
{
   utils::Logger* logger = QtRocket::getInstance()->getLogger();
   std::string name = m.data.commonName;
   if(motorModelMap.find(name) != motorModelMap.end())
   {
      logger->debug("Replacing MotorModel " + name + " in MotorModelDatabase");
   }
   else
   {
      logger->info("Adding MotorModel " + name + " to MotorModelDatabase");
   }
   motorModelMap[name] = m;
}

void MotorModelDatabase::addMotorModels(const std::vector<model::MotorModel>& models)
{
   utils::Logger* logger = QtRocket::getInstance()->getLogger();
   for(const auto& i : models)
   {
      addMotorModel(i);
   }
}

std::optional<model::MotorModel> MotorModelDatabase::getMotorModel(const std::string& name)
{
   auto mm = motorModelMap.find(name);
   if(mm == motorModelMap.end())
   {
      Logger::getInstance()->debug("Unable to locate " + name + " in MotorModel database");

      return std::nullopt;

   }
   else
   {
      Logger::getInstance()->debug("Retrieved " + name + " from MotorModel database");
      return motorModelMap[name];
   }
}

} // namespace utils
