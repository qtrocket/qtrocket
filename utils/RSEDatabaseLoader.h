#ifndef UTILS_RSEDATABASELOADER_H
#define UTILS_RSEDATABASELOADER_H

#include "model/MotorModel.h"

#include <vector>
#include <string>

#include <boost/property_tree/ptree.hpp>

namespace utils {

class RSEDatabaseLoader
{
public:
   RSEDatabaseLoader(const std::string& filename);
   ~RSEDatabaseLoader();

private:

   std::vector<MotorModel> motors;

   void buildAndAppendMotorModel(boost::property_tree::ptree& v);

   boost::property_tree::ptree tree;
};

} // namespace utils

#endif // UTILS_RSEDATABASELOADER_H
