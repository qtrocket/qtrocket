#ifndef UTILS_MOTORMODELDATABASE_H
#define UTILS_MOTORMODELDATABASE_H

/// \cond
// C headers
// C++ headers
#include <vector>
#include <string>
#include <map>
#include <optional>
#include <cstddef>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"


namespace utils
{

/**
 * @brief MotorQuery is a source-agnostic filter for selecting motors. Every field is optional; an
 *        unset field does not constrain the result, so a default-constructed MotorQuery matches
 *        everything. This is the unified replacement for per-source query notions: it folds in the
 *        old findMotorsByManufacturer / findMotorsByImpulseClass intent, and its fields map cleanly
 *        onto thrustcurve.org's SearchCriteria for when that source is added. Clients build a
 *        MotorQuery without knowing where the motors originated.
 */
struct MotorQuery
{
   std::optional<std::string> manufacturer; /// exact manufacturer name, e.g. "AeroTech"
   std::optional<std::string> impulseClass; /// motor letter, e.g. "A", "J", "1/2A"
   std::optional<double>      diameter;     /// motor diameter in mm (matched to within 0.5 mm)
   std::optional<std::string> nameContains; /// case-sensitive substring of the common name
};

/**
 * @brief MotorSummary is a lightweight, copyable description of a motor, sufficient to populate a
 *        list or combo box WITHOUT carrying the (potentially large, possibly not-yet-downloaded)
 *        thrust curve. Call getMotorModel(commonName) to obtain the complete model once a motor is
 *        actually selected. Splitting "list" from "fetch the full model" keeps listing cheap and
 *        lets a future remote source defer downloading the thrust curve until selection.
 */
struct MotorSummary
{
   std::string commonName;
   std::string manufacturer;
   double      avgThrust{0.0};    /// average thrust, Newtons
   double      totalImpulse{0.0}; /// total impulse, Newton-seconds
   double      diameter{0.0};     /// motor diameter, mm
   std::string impulseClass;      /// motor letter, e.g. "A", "J"
};

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
    * @brief importRSEFile parses a RockSim .rse engine file and adds every motor it contains to
    *        this database. This is the supported way to ingest RSE motors: callers (GUI, CLI) do
    *        not construct an RSEDatabaseLoader themselves and stay unaware of the file format.
    *
    * @param path filesystem path to a .rse file
    * @return the number of net new motors added to the database (motors whose common name was
    *         already present are replaced, not counted, so re-importing a file returns 0)
    * @throws std::exception (from the underlying XML parser) if the file cannot be read or parsed
    */
   std::size_t importRSEFile(const std::string& path);

   /**
    * @brief Get the Motor Model by Common Name
    *
    * @param name Motor Common name
    * @return std::optional<model::MotorModel>
    */
   std::optional<model::MotorModel> getMotorModel(const std::string& name);

   /**
    * @brief listMotors returns a lightweight summary of every motor matching the query. Iteration
    *        order follows the underlying map, so results are sorted by common name. Pass a
    *        default-constructed MotorQuery (the default argument) to list everything. This is the
    *        source-agnostic selection surface that subsumes findMotorsByManufacturer /
    *        findMotorsByImpulseClass.
    *
    * @param q source-agnostic filter; unset fields do not constrain the result
    * @return summaries of the matching motors, sorted by common name
    */
   std::vector<MotorSummary> listMotors(const MotorQuery& q = {}) const;

   /**
    * @brief size reports how many motors are currently stored, across every source ingested so far.
    * @return number of motors in the database
    */
   std::size_t size() const { return motorModelMap.size(); }

   void saveMotorDatabase(const std::string& filename);
   void loadMotorDatabase(const std::string& filename);
private:

   // Ingestion is internal: motors enter the database through importRSEFile (and future sources),
   // not by client code adding MotorModels directly. Adds replace any entry with the same common name.
   void addMotorModel(const model::MotorModel& m);
   void addMotorModels(const std::vector<model::MotorModel>& models);

   // The "database" is really just a map. :)
   /// motorModelMap is keyed off of the motor commonName
   std::map<std::string, model::MotorModel> motorModelMap;
   


};

} // namespace utils

#endif // UTILS_MOTORMODELDATABASE_H
