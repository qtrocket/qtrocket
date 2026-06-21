#ifndef MODEL_MOTORMODELDATABASE_H
#define MODEL_MOTORMODELDATABASE_H

/// \cond
// C headers
// C++ headers
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <optional>
#include <cstddef>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"


namespace model
{

// Owned internally so client code depends only on MotorModelDatabase, never on remote sources.
class ThrustCurveAPI;
class ThrustCurveClient;
class MotorModelDatabaseTestAccess;

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
 * @brief MotorSearchFacets enumerates the values an online search can be filtered by (the choices a
 *        UI would present). Source-agnostic; currently populated from thrustcurve.org metadata.
 */
struct MotorSearchFacets
{
   std::vector<std::string> manufacturers; /// manufacturer codes, e.g. "AeroTech"
   std::vector<double>      diameters;     /// motor diameters in mm
   std::vector<std::string> impulseClasses;/// motor letters, e.g. "A", "J"
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
    * @brief getOnlineSearchFacets returns the manufacturers / diameters / impulse classes that an
    *        online (thrustcurve.org) search can be filtered by. Performs a network request.
    * @return the available search facets (empty if the request fails)
    */
   MotorSearchFacets getOnlineSearchFacets();

   /**
    * @brief searchOnline queries thrustcurve.org for motors matching the query, merges the results
    *        into this database (so getMotorModel()/listMotors() then see them) and returns their
    *        summaries. Performs network requests; clients never touch ThrustCurveClient directly.
    * @param q source-agnostic filter (manufacturer / impulseClass / diameter; nameContains ignored)
    * @return summaries of the matching motors (empty if the request fails or matches nothing)
    */
   std::vector<MotorSummary> searchOnline(const MotorQuery& q);

   /**
    * @brief size reports how many motors are currently stored, across every source ingested so far.
    * @return number of motors in the database
    */
   std::size_t size() const { return motorModelMap.size(); }

   void saveMotorDatabase(const std::string& filename);
   void loadMotorDatabase(const std::string& filename);
private:
   // Private test seam only: MotorModelDatabaseTestAccess injects a fake remote source so the
   // online wrapper methods can be unit-tested without network calls. Production code uses the
   // public default constructor and lazily gets a real ThrustCurveClient.
   explicit MotorModelDatabase(std::unique_ptr<ThrustCurveAPI> thrustCurveApi);
   friend class MotorModelDatabaseTestAccess;

   // Ingestion is internal: motors enter the database through importRSEFile (and future sources),
   // not by client code adding MotorModels directly. Adds replace any entry with the same common name.
   void addMotorModel(const model::MotorModel& m);
   void addMotorModels(const std::vector<model::MotorModel>& models);

   /// Build the lightweight summary used by listMotors()/searchOnline().
   static MotorSummary toSummary(const model::MotorModel& m);

   /// Lazily construct (on first online use) and return the owned thrustcurve.org client.
   ThrustCurveAPI& thrustCurveApi();

   // The "database" is really just a map. :)
   /// motorModelMap is keyed off of the motor commonName
   std::map<std::string, model::MotorModel> motorModelMap;

   /// thrustcurve.org client, owned so clients never depend on it. Null until first online use.
   std::unique_ptr<ThrustCurveAPI> tcApi;
};

} // namespace model

#endif // MODEL_MOTORMODELDATABASE_H
