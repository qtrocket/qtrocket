#ifndef MODEL_DESIGNSERIALIZER_H
#define MODEL_DESIGNSERIALIZER_H

/// \cond
// C++ headers
#include <string>
/// \endcond

namespace model
{

class RocketModel;
class MotorModelDatabase;

/**
 * @brief Versioned save/load of a RocketModel design (part tree + motor-by-name + aero options) to a
 *        Boost.PropertyTree XML file (.qrd), mirroring MotorModelDatabase's idiom.
 *
 * The part tree is stored polymorphically as
 *   <part type=".." name=".."><params .../><offset x= y= z=/><children>...</children></part>
 * where @c type is the part's typeName(), the params come from the factory reflector
 * (model::part::params), and each part's VERBATIM CM-to-CM attach offset is stored and replayed
 * exactly -- so a reloaded design reproduces the original mass and CG. The motor is stored by common
 * name only and re-resolved on load against the supplied MotorModelDatabase (a missing motor is a
 * logged warning, not an error -- the flight-faithful guarantee holds only against the same DB).
 *
 * load() installs the rebuilt tree IN PLACE via RocketModel::setRoot, then applies the aero/sim
 * options, so a restored manual reference-area override wins over setRoot's reset of that flag.
 */
class DesignSerializer
{
public:
   /// @brief Write @p rocket's design to @p filename as versioned XML. @throws on a write error.
   static void save(const RocketModel& rocket, const std::string& filename);

   /// @brief Load a design from @p filename into @p rocket IN PLACE (via setRoot), re-resolving the
   ///        motor by common name against @p motors.
   /// @throws std::runtime_error on an unsupported major version or a failed attach, and propagates
   ///         Boost (bad/malformed file) and concrete-ctor (bad geometry) exceptions.
   static void load(RocketModel& rocket, MotorModelDatabase& motors, const std::string& filename);
};

} // namespace model

#endif // MODEL_DESIGNSERIALIZER_H
