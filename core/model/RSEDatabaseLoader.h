#ifndef MODEL_RSEDATABASELOADER_H
#define MODEL_RSEDATABASELOADER_H


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

namespace model {

class RSEDatabaseLoader
{
public:
   RSEDatabaseLoader(const std::string& filename);
   ~RSEDatabaseLoader();

   std::vector<model::MotorModel>& getMotors() { return motors; }

   model::MotorModel getMotorModelByName(const std::string& name);
private:

   std::vector<model::MotorModel> motors;

   void buildAndAppendMotorModel(boost::property_tree::ptree& v);

   boost::property_tree::ptree tree;
};

} // namespace model

#endif // MODEL_RSEDATABASELOADER_H
