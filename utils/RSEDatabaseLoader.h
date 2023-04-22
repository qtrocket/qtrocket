#ifndef UTILS_RSEDATABASELOADER_H
#define UTILS_RSEDATABASELOADER_H


/// \cond
// C headers
// C++ headers
#include <string>
#include <vector>

// 3rd party headers
#include <boost/property_tree/ptree.hpp>
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"

namespace utils {

class RSEDatabaseLoader
{
public:
   RSEDatabaseLoader(const std::string& filename);
   ~RSEDatabaseLoader();

   const std::vector<model::MotorModel>& getMotors() const { return motors; }
private:

   std::vector<model::MotorModel> motors;

   void buildAndAppendMotorModel(boost::property_tree::ptree& v);

   boost::property_tree::ptree tree;
};

} // namespace utils

#endif // UTILS_RSEDATABASELOADER_H
