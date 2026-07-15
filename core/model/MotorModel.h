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
 * @brief Holds a hobby rocket motor's data -- manufacturer, burn time, max thrust, propellant
 *        weight, etc. -- plus a ThrustCurve of its thrust samples.
 */
class MotorModel
{
public:
    MotorModel();
    MotorModel(const MotorModel&) = default;
    MotorModel(MotorModel&&) = default;
    ~MotorModel();

    MotorModel& operator=(const MotorModel&) = default;
    MotorModel& operator=(MotorModel&&) = default;

    /// Whether a motor is out of production or still available.
    enum class AVAILABILITY
    {
        REGULAR,    /// available
        OCCASIONAL, /// in production, made in occasional runs
        OOP         /// Out of Production
    };


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

    /// Certification organization that certified the motor.
    enum class CERTORG
    {
        AMRS, /// Australian Model Rocket Society
        CAR, /// Canadian Association of Rocketry
        NAR, /// National Association of Rocketry
        TRA, /// Tripoli
        UNC, /// Uncertified
        UNK  /// Unknown Certification
    };

    /// Motor type: single-use, reload, or hybrid.
    enum class MOTORTYPE
    {
        SU,
        RELOAD,
        HYBRID
    };

    /// AVAILABILITY plus a string-name round-trip (str/toEnum).
    struct MotorAvailability
    {
        MotorAvailability(const AVAILABILITY& a) : availability(a) {}
        MotorAvailability(const MotorAvailability&) = default;
        MotorAvailability(MotorAvailability&&) = default;
        MotorAvailability() : MotorAvailability(AVAILABILITY::REGULAR) {}

        MotorAvailability& operator=(const MotorAvailability&) = default;
        MotorAvailability& operator=(MotorAvailability&&) = default;

        AVAILABILITY availability{AVAILABILITY::REGULAR};
        std::string str() const
        {
            if(availability == AVAILABILITY::REGULAR)
                return std::string("regular");
            else if(availability == AVAILABILITY::OCCASIONAL)
                return std::string("occasional");
            else
                return std::string("OOP");
        }

        static AVAILABILITY toEnum(const std::string& name)
        {
            if(name == "regular")
                return AVAILABILITY::REGULAR;
            else if(name == "occasional")
                return AVAILABILITY::OCCASIONAL;
            else
                return AVAILABILITY::OOP;
        }
    };

    /// CERTORG plus a string-name round-trip (str/toEnum).
    struct CertOrg
    {
        CertOrg(const CERTORG& c) : org(c) {}
        CertOrg(const CertOrg&) = default;
        CertOrg(CertOrg&&) = default;
        CertOrg() : CertOrg(CERTORG::UNC) {}

        CertOrg& operator=(const CertOrg&) = default;
        CertOrg& operator=(CertOrg&&) = default;

        CERTORG org{CERTORG::UNC};
        std::string str() const
        {
            if(org == CERTORG::AMRS)
                return std::string("Australian Model Rocket Society Inc.");
            else if(org == CERTORG::CAR)
                return std::string("Canadian Association of Rocketry");
            else if(org == CERTORG::NAR)
                return std::string("National Association of Rocketry");
            else if(org == CERTORG::TRA)
                return std::string("Tripoli Rocketry Association, Inc.");
            else if(org == CERTORG::UNC)
                return std::string("Uncertified");
            else // UNK - Unknown
                return std::string("Unknown");
        }

        static CERTORG toEnum(const std::string& name)
        {
            // Accept the short codes and the full names emitted by str(), so a str()->toEnum()
            // round-trip is lossless.
            if(name == "AMRS" || name == "Australian Model Rocket Society Inc.")
                return CERTORG::AMRS;
            else if(name == "CAR" || name == "Canadian Association of Rocketry")
                return CERTORG::CAR;
            else if(name == "NAR" || name == "National Association of Rocketry")
                return CERTORG::NAR;
            else if(name == "TRA" || name == "Tripoli Rocketry Association, Inc.")
                return CERTORG::TRA;
            else if(name == "UNC" || name == "Uncertified")
                return CERTORG::UNC;
            else // Unknown
                return CERTORG::UNK;

        }
    };

    /// MOTORTYPE plus a string-name round-trip (str/toEnum).
    struct MotorType
    {
        MotorType(const MOTORTYPE& t) : type(t) {}
        MotorType(const MotorType&) = default;
        MotorType(MotorType&&) = default;
        MotorType() : MotorType(MOTORTYPE::SU) {}

        MotorType& operator=(const MotorType&) = default;
        MotorType& operator=(MotorType&&) = default;

        MOTORTYPE type;
        std::string str() const
        {
            if(type == MOTORTYPE::SU)
                return std::string("Single Use");
            else if(type == MOTORTYPE::RELOAD)
                return std::string("Reload");
            else
                return std::string("Hybrid");
        }

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

    /// MOTORMANUFACTURER plus a string-name round-trip (str/toEnum).
    struct MotorManufacturer
    {
        MotorManufacturer(const MOTORMANUFACTURER& m) : manufacturer(m) {}
        MotorManufacturer() : manufacturer(MOTORMANUFACTURER::UNKNOWN) {}
        MotorManufacturer(const MotorManufacturer&) = default;
        MotorManufacturer(MotorManufacturer&&) = default;

        MotorManufacturer& operator=(const MotorManufacturer&) = default;
        MotorManufacturer& operator=(MotorManufacturer&&) = default;

        MOTORMANUFACTURER manufacturer;
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

        static MOTORMANUFACTURER toEnum(const std::string& name)
        {
            // Accepts short codes, the full names thrustcurve.org returns, and a few legacy variants, so
            // one helper maps RSE, thrustcurve.org, and saved-DB manufacturers. Unknown ones -> UNKNOWN.
            if(name == "AeroTech" ||
                 name == "Aerotech")
                return MOTORMANUFACTURER::AEROTECH;
            else if(name == "AMW" ||
                       name == "Animal Motor Works")
                return MOTORMANUFACTURER::AMW;
            else if(name == "Cesaroni" ||
                       name == "Cesaroni Technology" ||
                       name == "Cesaroni Technology Inc.")
                return MOTORMANUFACTURER::CESARONI;
            else if(name == "Estes" ||
                       name == "Estes Industries" ||
                       name == "Estes Industries, Inc.")
                return MOTORMANUFACTURER::ESTES;
            else if(name == "Loki" ||
                       name == "Loki Research")
                return MOTORMANUFACTURER::LOKI;
            else if(name == "Apogee" ||
                       name == "Apogee Components")
                return MOTORMANUFACTURER::APOGEE;
            else if(name == "Contrail" ||
                       name == "Contrail Rockets")
                return MOTORMANUFACTURER::CONTRAIL;
            else if(name == "Hypertek")
                return MOTORMANUFACTURER::HYPERTEK;
            else if(name == "Klima" ||
                       name == "Raketenmodellbau Klima")
                return MOTORMANUFACTURER::KLIMA;
            else if(name == "Quest" ||
                       name == "Quest Aerospace")
                return MOTORMANUFACTURER::QUEST;
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

        /// Derive the impulse class prefix from a motor code such as "G80T", "1/2A3", or "1/4A2".
        static std::string deriveImpulseClass(const std::string& motorCode);

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
        double propWeight{0.0};   /// Propellant weight in kg (loaders convert; consumed as kg by getMass/computeMassCurve)
        bool sparky{false};       /// true if the motor is "sparky", false otherwise
        double totalImpulse{0.0}; /// Total impulse in Newton-seconds
        double totalWeight{0.0};  /// Total weight in kg (loaders convert; consumed as kg by getMass/computeMassCurve)
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
    double emptyMass{0.0};
    double isp{0.0};
    double ignitionTime{0.0}; /// 0 until startMotor(); read by getThrust's burnout test, so it must
                                       /// be initialized -- garbage here makes the flight depend on stack/heap layout
    ThrustCurve thrust; /// The measured motor thrust curve

    std::vector<std::pair<double, double>> massCurve;

    void computeMassCurve();
   
};

} // namespace model

#endif // MODEL_MOTORMODEL_H
