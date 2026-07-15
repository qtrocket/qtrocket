#ifndef MODEL_THRUSTCURVECLIENT_H
#define MODEL_THRUSTCURVECLIENT_H


/// \cond
// C headers
// C++ headers
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/CurlConnection.h"
#include "model/MotorModel.h"

namespace model
{

class ThrustcurveMetadata
{
public:
    ThrustcurveMetadata() = default;
    ~ThrustcurveMetadata() = default;

    ThrustcurveMetadata(const ThrustcurveMetadata&) = default;
    ThrustcurveMetadata(ThrustcurveMetadata&&) = default;

    ThrustcurveMetadata& operator=(const ThrustcurveMetadata&) = default;
    ThrustcurveMetadata& operator=(ThrustcurveMetadata&&) = default;

//private:
    std::vector<model::MotorModel::CertOrg> certOrgs;
    std::vector<double> diameters;
    std::vector<std::string> impulseClasses;
    std::map<std::string, std::string> manufacturers;
    std::vector<model::MotorModel::MotorType> types;

};

class SearchCriteria
{
public:
    SearchCriteria() = default;
    ~SearchCriteria() = default;
    SearchCriteria(const SearchCriteria&) = default;
    SearchCriteria(SearchCriteria&&) = default;

    SearchCriteria& operator=(const SearchCriteria&) = default;
    SearchCriteria& operator=(SearchCriteria&&) = default;

    void addCriteria(const std::string& name,
                           const std::string& value);

    std::map<std::string, std::string> criteria;

};

/**
 * @brief Internal remote-motor-source interface used by MotorModelDatabase. Exists so the online
 *        wrappers can be tested with a fake source; the injection point is private/friend-only in
 *        MotorModelDatabase, so application code never uses this directly.
 */
class ThrustCurveAPI
{
public:
    virtual ~ThrustCurveAPI() = default;

    virtual ThrustcurveMetadata getMetadata() = 0;
    virtual std::vector<model::MotorModel> searchMotors(const SearchCriteria& c) = 0;
};

/**
 * @brief Parsed payload of a search.json response. Carries metadata only;
 *        thrust curves are fetched separately per motor via download.json.
 */
struct SearchResponse
{
    std::vector<model::MotorModel::MetaData> motors;
    int matches{0}; ///< server-reported total; may exceed motors.size()
};

// Pure parsers over raw thrustcurve.org response bodies, split out from the
// HTTP layer so they can be unit-tested with canned JSON. nullopt means the
// body was malformed (logged); a response whose "error" field is set is
// logged and yields empty-but-valid results.
std::optional<SearchResponse> parseSearchResponse(const std::string& json);
std::optional<ThrustcurveMetadata> parseMetadataResponse(const std::string& json);
/// Selects one simfile's samples: the first RASP entry with samples, else the first entry of any
/// format with samples. Never concatenates simfiles.
std::optional<std::vector<std::pair<double, double>>>
parseDownloadResponse(const std::string& json);

/// @brief Production thrustcurve.org HTTP client used by MotorModelDatabase.
class ThrustCurveClient
    : public ThrustCurveAPI
{
public:
    ThrustCurveClient();
    ~ThrustCurveClient() override;

    ThrustcurveMetadata getMetadata() override;

    std::vector<model::MotorModel> searchMotors(const SearchCriteria& c) override;



private:

    const std::string hostname;
    utils::CurlConnection curlConnection;

    std::optional<ThrustCurve> getThrustCurve(const std::string& id);
};

} // namespace model

#endif // MODEL_THRUSTCURVECLIENT_H
