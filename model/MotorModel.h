#ifndef MODEL_MOTORMODEL_H
#define MODEL_MOTORMODEL_H

/// \cond
// C headers
// C++ headers
#include <string>

// 3rd party headers
// For boost serialization. We're using boost::serialize to save
// and load Motor data to file. (CURRENTLY UNUSED)
//#include <boost/archive/text_iarchive.hpp>
//#include <boost/archive/text_oarchive.hpp>
/// \endcond

// qtrocket theaders
#include "ThrustCurve.h"

namespace model
{

/**
 * @brief The MotorModel class
 *
 * The MotorModel class defines a structure that holds data relating to a hobby
 * rocket motor such as the manufacturer, burn time, maximum thrust, propellant
 * weight, etc. It also holds a ThrustCurve object that contains thrust sample data
 * for that motor.
 *
 * There are several additional classes defined within the MotorModel class designed
 * to encapsulate and define several pieces of motor related data as well.
 */
class MotorModel
{
public:
    /**
    * @brief MotorModel constructor
    */
   MotorModel();
    /**
    * @brief MotorModel copy constructor is defaulted
    */
   MotorModel(const MotorModel&) = default;
    /**
    * @brief MotorModel move constructor is defaulted
    */
   MotorModel(MotorModel&&) = default;
   ~MotorModel();

   /**
    * @brief Copy assignment operator is defaulted
    * @return Copy of MotorModel
    */
   MotorModel& operator=(const MotorModel&) = default;

   /**
    * @brief Move assignment operator is defaulted
    * @return Moved MotorModel
    */
   MotorModel& operator=(MotorModel&&) = default;

   /**
    * @brief The AVAILABILITY enum class identifies whether a motor is
    *        out of production, or still available
    */
   enum class AVAILABILITY
   {
      REGULAR, /// available
      OOP      /// Out of Production
   };


   /**
    * @brief The MOTORMANUFACTURER enum class identifies the motor
    *        manufacturer
    */
   enum class MOTORMANUFACTURER
   {
       AEROTECH,
       AMW,
       APOGEE,
       CESARONI,
       CONTRAIL,
       ESTES,
       HYPERTEK,
       KLIMA,
       LOKI,
       QUEST,
       UNKNOWN
   };

   /**
    * @brief The CERTORG enum class identifies the Certification Organization
    *        that certified the motor
    */
   enum class CERTORG
   {
      AMRS, /// Australian Model Rocket Society
      CAR, /// Canadian Association of Rocketry
      NAR, /// National Association of Rocketry
      TRA, /// Tripoli
      UNC, /// Uncertified
      UNK  /// Unknown Certification
   };

   /**
    * @brief The MOTORTYPE enum class identifies the motor type, either
    *        Single-Use, Reload, or Hybrid
    */
   enum class MOTORTYPE
   {
      SU,
      RELOAD,
      HYBRID
   };

   /**
    * @brief The MotorAvailability struct wraps the AVAILABILITY enum and
    *        provides a helper function to return a string representation
    *        of the AVAILABILITY enum.
    */
   struct MotorAvailability
   {
      MotorAvailability(const AVAILABILITY& a) : availability(a) {}
      MotorAvailability(const MotorAvailability&) = default;
      MotorAvailability(MotorAvailability&&) = default;
      MotorAvailability() : MotorAvailability(AVAILABILITY::REGULAR) {}

      MotorAvailability& operator=(const MotorAvailability&) = default;
      MotorAvailability& operator=(MotorAvailability&&) = default;

      AVAILABILITY availability{AVAILABILITY::REGULAR};
      /**
       * @brief str Returns a string representation of AVAILABILITY enum
       * @return string representation
       */
      std::string str() const
      {
         if(availability == AVAILABILITY::REGULAR)
            return std::string("regular");
         else
            return std::string("OOP");
      }

      /**
       * @brief toEnum returns AVAILABILITY enum from string name
       * @param name Name of enum
       * @return AVAILABILITY enum corresponding to name
       */
      static AVAILABILITY toEnum(const std::string& name)
      {
         if(name == "regular")
            return AVAILABILITY::REGULAR;
         else
            return AVAILABILITY::OOP;
      }
   };

   /**
    * @brief The CertOrg struct wraps the CERTORG enum and
    *        provides a helper function to return a string representation
    *        of the CERTORG enum.
    */
   struct CertOrg
   {
      CertOrg(const CERTORG& c) : org(c) {}
      CertOrg(const CertOrg&) = default;
      CertOrg(CertOrg&&) = default;
      CertOrg() : CertOrg(CERTORG::UNC) {}

      CertOrg& operator=(const CertOrg&) = default;
      CertOrg& operator=(CertOrg&&) = default;

      CERTORG org{CERTORG::UNC};
      /**
       * @brief str Returns a string representation of CERTORG enum
       * @return string representation
       */
      std::string str() const
      {
         if(org == CERTORG::AMRS)
            return std::string("Austrialian Model Rocket Society Inc.");
         else if(org == CERTORG::CAR)
            return std::string("Canadian Association of Rocketry");
         else if(org == CERTORG::NAR)
            return std::string("National Association of Rocketry");
         else if(org == CERTORG::TRA)
            return std::string("Tripoli Rocketry Association, Inc.");
         else if(org == CERTORG::UNC)
            return std::string("Uncertified");
         else // UNK - Unknown
            return std::string("Unkown");
      }

      /**
       * @brief toEnum returns CERTORG enum corresponding to name
       * @param name Name of enumeration
       * @return enumeration value corresponding to name
       */
      static CERTORG toEnum(const std::string& name)
      {
         if(name == "AMRS")
            return CERTORG::AMRS;
         else if(name == "CAR")
            return CERTORG::CAR;
         else if(name == "NAR")
            return CERTORG::NAR;
         else if(name == "TRA")
            return CERTORG::TRA;
         else if(name == "UNC")
            return CERTORG::UNC;
         else // Unknown
            return CERTORG::UNK;

      }
   };

   /**
    * @brief The MotorType struct wraps the MOTORTYPE enum and
    *        provides a helper function to return a string representation
    *        of the MOTORTYPE enum.
    */
   struct MotorType
   {
      MotorType(const MOTORTYPE& t) : type(t) {}
      MotorType(const MotorType&) = default;
      MotorType(MotorType&&) = default;
      MotorType() : MotorType(MOTORTYPE::SU) {}

      MotorType& operator=(const MotorType&) = default;
      MotorType& operator=(MotorType&&) = default;

      MOTORTYPE type;
      /**
       * @brief str Returns a string representation of MOTORTYPE enum
       * @return string representation
       */
      std::string str() const
      {
         if(type == MOTORTYPE::SU)
            return std::string("Single Use");
         else if(type == MOTORTYPE::RELOAD)
            return std::string("Reload");
         else
            return std::string("Hybrid");
      }

      /**
       * @brief toEnum returns enumeration corresponding to name
       * @param name Name of enumeration
       * @return enumeration corresponding to name
       */
      static MOTORTYPE toEnum(const std::string& name)
      {
         if(name == "SU" ||
             name == "Single Use" ||
             name == "single-use")
            return MOTORTYPE::SU;
         else if(name == "reload" ||
                  name == "Reload" || 
                  name == "reloadable")
            return MOTORTYPE::RELOAD;
         else  // It's a hybrid
            return MOTORTYPE::HYBRID;

      }
   };

   /**
    * @brief The MotorManufacturer struct wraps the MOTORMANUFACTURER enum and
    *        provides a helper function to return a string representation
    *        of the MOTORMANUFACTURER enum.
    */
   struct MotorManufacturer
   {
      MotorManufacturer(const MOTORMANUFACTURER& m) : manufacturer(m) {}
      MotorManufacturer() : manufacturer(MOTORMANUFACTURER::UNKNOWN) {}
      MotorManufacturer(const MotorManufacturer&) = default;
      MotorManufacturer(MotorManufacturer&&) = default;

      MotorManufacturer& operator=(const MotorManufacturer&) = default;
      MotorManufacturer& operator=(MotorManufacturer&&) = default;

      MOTORMANUFACTURER manufacturer;
      /**
       * @brief str Returns a string representation of MOTORMANUFACTURER enum
       * @return string representation
       */
      std::string str() const
      {
         switch(manufacturer)
         {
         case MOTORMANUFACTURER::AEROTECH:
            return std::string("AeroTech");
         case MOTORMANUFACTURER::AMW:
            return std::string("AMW");
         case MOTORMANUFACTURER::CESARONI:
            return std::string("Cesaroni");
         case MOTORMANUFACTURER::ESTES:
            return std::string("Estes");
         case MOTORMANUFACTURER::LOKI:
            return std::string("Loki");
         case MOTORMANUFACTURER::APOGEE:
            return std::string("Apogee");
         case MOTORMANUFACTURER::CONTRAIL:
            return std::string("Contrail");
         case MOTORMANUFACTURER::HYPERTEK:
            return std::string("Hypertek");
         case MOTORMANUFACTURER::KLIMA:
            return std::string("Klima");
         case MOTORMANUFACTURER::QUEST:
            return std::string("Quest");
         case MOTORMANUFACTURER::UNKNOWN:
         default:
            return std::string("Unknown");
         }
      }

      /**
       * @brief toEnum returns MOTORMANUFACTURER enum value corresponding to a name
       * @param name Name of enumeration
       * @return enumeration corresponding to name
       */
      static MOTORMANUFACTURER toEnum(const std::string& name)
      {
         if(name == "AeroTech" ||
             name == "Aerotech")
            return MOTORMANUFACTURER::AEROTECH;
         else if(name == "AMW" ||
                 name == "Animal Motor Works")
            return MOTORMANUFACTURER::AMW;
         else if(name == "Cesaroni" ||
                 name == "Cesaroni Technology Inc.")
            return MOTORMANUFACTURER::CESARONI;
         else if(name == "Estes" ||
                 name == "Estes Industries, Inc.")
            return MOTORMANUFACTURER::ESTES;
         else if(name == "Loki")
            return MOTORMANUFACTURER::LOKI;
         else if(name == "Apogee")
            return MOTORMANUFACTURER::APOGEE;
         else if(name == "Contrail")
            return MOTORMANUFACTURER::CONTRAIL;
         else if(name == "Hypertek")
            return MOTORMANUFACTURER::HYPERTEK;
         else if(name == "Klima")
            return MOTORMANUFACTURER::QUEST;
         else if(name == "Quest")
            return MOTORMANUFACTURER::KLIMA;
         else
            return MOTORMANUFACTURER::UNKNOWN;
      }
   };


/// TODO: make these MotorModel members private. Public just for testing
//private:

   struct MetaData
   {
      MetaData() = default;
      ~MetaData() = default;
      MetaData(const MetaData&) = default;
      MetaData(MetaData&&) = default;
      MetaData& operator=(const MetaData&) = default;
      MetaData& operator=(MetaData&&) = default;

      MotorAvailability availability{AVAILABILITY::REGULAR}; /// Motor Availability
      double avgThrust{0.0}; /// Average thrust in Newtons
      double burnTime{0.0};  /// Burn time in seconds
      CertOrg certOrg{CERTORG::UNC}; /// The certification organization, defaults to Uncertified
      std::string commonName{""}; /// Common name, e.g. A8 or J615
      // int dataFiles
      std::vector<int> delays; /// 1000 delay means no ejection charge
      std::string designation{""}; /// Other name, usually includes prop type, e.g. H110W
      double diameter{0}; /// motor diameter in mm
      std::string impulseClass; /// Motor letter, e.g. 'A', 'B', '1/2A', 'M', etc
      std::string infoUrl{""};  /// TODO: ???
      double length{0.0}; /// motor length in mm
      MotorManufacturer manufacturer{MOTORMANUFACTURER::UNKNOWN}; /// Motor Manufacturer

      double maxThrust{0.0}; /// Max thrust in Newtons
      std::string motorIdTC{""}; /// 24 character hex string used by thrustcurve.org to ID a motor
      std::string propType{""}; /// Propellant type, e.g. black powder
      double propWeight{0.0};   /// Propellant weight in grams
      bool sparky{false};       /// true if the motor is "sparky", false otherwise
      double totalImpulse{0.0}; /// Total impulse in Newton-seconds
      double totalWeight{0.0};  /// Total weight in grams
      MotorType type{MOTORTYPE::SU}; /// Motor type, e.g. single-use, reload, or hybrid
      std::string lastUpdated{""}; /// Date last updated on ThrustCurve.org
   };

   double getMass(double simTime) const;
   double getThrust(double simTime);

   void setMetaData(const MetaData& md);
   void moveMetaData(MetaData&& md);

   void startMotor(double startTime) { ignitionOccurred = true; ignitionTime = startTime; }

   void addThrustCurve(const ThrustCurve& tc) { thrust = tc; }

   const ThrustCurve& getThrustCurve() const { return thrust; }

   // Thrust parameters
   MetaData data;
private:
   bool ignitionOccurred{false};
   bool burnOutOccurred{false};
   double emptyMass;
   double isp;
   double maxTime;
   double ignitionTime;
   ThrustCurve thrust; /// The measured motor thrust curve

   std::vector<std::pair<double, double>> massCurve;

   void computeMassCurve();
   
};

} // namespace model

#endif // MODEL_MOTORMODEL_H
