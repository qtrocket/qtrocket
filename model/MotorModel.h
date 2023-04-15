#ifndef MODEL_MOTORMODEL_H
#define MODEL_MOTORMODEL_H

// For boost serialization. We're using boost::serialize to save
// and load Motor data to file
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <string>

#include "Thrustcurve.h"
#include "MotorCase.h"

class MotorModel
{
public:
   MotorModel();
   ~MotorModel();

   void setDataFromJsonString(const std::string& jsonString);

   enum class AVAILABILITY
   {
      REGULAR, // available
      OOP      // Out of Production
   };

   enum class CERTORG
   {
      AMRS,
      CAR,
      NAR,
      TRA,
      UNC
   };

   enum class MOTORTYPE
   {
      SU,
      RELOAD,
      HYBRID
   };

   struct MotorAvailability
   {
      MotorAvailability(const AVAILABILITY& a) : availability(a) {}
      MotorAvailability() : MotorAvailability(AVAILABILITY::REGULAR) {}

      AVAILABILITY availability{AVAILABILITY::REGULAR};
      std::string str()
      {
         if(availability == AVAILABILITY::REGULAR)
            return std::string("regular");
         else
            return std::string("OOP");
      }
   };

   struct CertOrg
   {
      CertOrg(const CERTORG& c) : org(c) {}
      CertOrg() : CertOrg(CERTORG::UNC) {}

      CERTORG org{CERTORG::UNC};
      std::string str()
      {
         if(org == CERTORG::AMRS)
            return std::string("Austrialian Model Rocket Society Inc.");
         else if(org == CERTORG::CAR)
            return std::string("Canadian Association of Rocketry");
         else if(org == CERTORG::NAR)
            return std::string("National Association of Rocketry");
         else if(org == CERTORG::TRA)
            return std::string("Tripoli Rocketry Association, Inc.");
         else // Uncertified
            return std::string("Uncertified");
      }
   };

   struct MotorType
   {
      MotorType(const MOTORTYPE& t) : type(t) {}
      MotorType() : MotorType(MOTORTYPE::SU) {}

      MOTORTYPE type;
      std::string str()
      {
         if(type == MOTORTYPE::SU)
            return std::string("Single Use");
         else if(type == MOTORTYPE::RELOAD)
            return std::string("Reload");
         else
            return std::string("Hybrid");
      }
   };


private:
   // Needed for boost serialize
   friend class boost::serialization::access;
   template<class Archive>
   void serialize(Archive& ar, const unsigned int version);

   MotorAvailability availability{AVAILABILITY::REGULAR};
   double avgThrust{0.0};
   double burnTime{0.0};
   CertOrg certOrg{CERTORG::UNC};
   std::string commonName{""};
   // int dataFiles
   std::vector<int> delays; // -1 delay means no ejection charge
   std::string designation{""};
   int diameter{0};
   std::string impulseClass; // 'A', 'B', '1/2A', 'M', etc
   std::string infoUrl{""};
   int length;
   std::string manufacturer;

   double maxThrust{0.0};
   std::string motorIdTC{""}; // 24 character hex string used by thrustcurve to ID a motor
   std::string propType{""}; // black powder, etc
   double propWeight{0.0};
   bool sparky{false};
   double totalImpulse;
   double totalWeight;
   MotorType type{MOTORTYPE::SU};
   std::string lastUpdated{""};

   // Thrust parameters
   Thrustcurve thrust;
   
   // Physical dimensions

};

template<class Archive>
void MotorModel::serialize(Archive& ar, const unsigned int version)
{

   ar & manufacturer;
   ar & impulseClass;
   ar & propType;
   ar & sparky;
   ar & totalImpulse;
   ar & delays;
   ar & burnTime;
   ar & thrust;
   ar & diameter;
   ar & length;
   ar & totalWeight;
   ar & propWeight;
}

#endif // MODEL_MOTORMODEL_H
