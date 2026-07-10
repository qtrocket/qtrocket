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
 * @brief Source-agnostic filter for selecting motors. Every field is optional; an unset field does
 *        not constrain, so a default-constructed MotorQuery matches everything. Its fields map onto
 *        thrustcurve.org's SearchCriteria.
 */
struct MotorQuery
{
   std::optional<std::string> manufacturer; /// exact manufacturer name, e.g. "AeroTech"
   std::optional<std::string> impulseClass; /// motor letter, e.g. "A", "J", "1/2A"
   std::optional<double>      diameter;     /// motor diameter in mm (matched to within 0.5 mm)
   std::optional<std::string> nameContains; /// case-sensitive substring of the common name
};

/**
 * @brief Lightweight, copyable description of a motor for populating a list, without the
 *        (potentially large, not-yet-downloaded) thrust curve. Call getMotorModel(commonName) for
 *        the full model on selection -- which lets a remote source defer the download until then.
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
 * @brief The values an online search can be filtered by (the choices a UI presents). Currently
 *        populated from thrustcurve.org metadata.
 */
struct MotorSearchFacets
{
   std::vector<std::string> manufacturers; /// manufacturer codes, e.g. "AeroTech"
   std::vector<double>      diameters;     /// motor diameters in mm
   std::vector<std::string> impulseClasses;/// motor letters, e.g. "A", "J"
};

/// @brief Storage, search, and retrieval of model rocket motors; the single ingest/selection surface.
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


   /// Parse a RockSim .rse file and add its motors. The supported RSE ingest path; callers stay
   /// unaware of the file format.
   /// @return net new motors added (same-name entries are replaced, not counted, so a re-import is 0)
   /// @throws std::exception (from the XML parser) if the file can't be read or parsed
   std::size_t importRSEFile(const std::string& path);

   /// Parse a RASP .eng file and add its motors; mirrors importRSEFile.
   /// @return net new motors added (see importRSEFile)
   /// @throws std::exception if the file can't be read or parsed
   std::size_t importRASPFile(const std::string& path);

   /// The motor model with this common name, or nullopt if absent.
   std::optional<model::MotorModel> getMotorModel(const std::string& name);

   /// Summaries of every motor matching @p q, sorted by common name (default-constructed q lists
   /// everything). The source-agnostic selection surface.
   std::vector<MotorSummary> listMotors(const MotorQuery& q = {}) const;

   /// The facets an online (thrustcurve.org) search can filter by. Performs a network request;
   /// empty if it fails.
   MotorSearchFacets getOnlineSearchFacets();

   /// Query thrustcurve.org, merge the results into this database (so getMotorModel()/listMotors()
   /// see them), and return their summaries. Network request; @p q.nameContains is ignored here.
   std::vector<MotorSummary> searchOnline(const MotorQuery& q);

   /// Number of motors stored across every ingested source.
   std::size_t size() const { return motorModelMap.size(); }

   void saveMotorDatabase(const std::string& filename);
   void loadMotorDatabase(const std::string& filename);
private:
   // Test seam: injects a fake remote source so the online wrappers can be tested without network.
   explicit MotorModelDatabase(std::unique_ptr<ThrustCurveAPI> thrustCurveApi);
   friend class MotorModelDatabaseTestAccess;

   // Internal ingestion (not client-facing). Adds replace any entry with the same common name.
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
