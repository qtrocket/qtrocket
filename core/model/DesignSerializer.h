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
 *        Boost.PropertyTree XML file, mirroring MotorModelDatabase's idiom.
 *
 * The part tree is stored polymorphically as
 *   <part type=".." name=".."><params .../><link .../><children>...</children></part>
 * with type = typeName(), params from the factory reflector (model::part::params), and the
 * StationLink placement intent in <link> (seat, parentStation, childStation, gap). A default-equal
 * link is elided and recovered as the zero-config abut default, so a reload reproduces geometry,
 * mass, and CG. The motor is stored by common name and re-resolved on load against the supplied
 * database (a miss is a logged warning, not an error -- faithfulness holds only against the same DB).
 */
class DesignSerializer
{
public:
    /// Write @p rocket's design to @p filename as versioned XML. @throws on a write error.
    static void save(const RocketModel& rocket, const std::string& filename);

    /// Load a design from @p filename and install it into @p rocket (via installDesign, atomically),
    /// re-resolving the motor by common name against @p motors.
    /// @throws std::runtime_error on an unsupported major version or a failed attach; propagates Boost
    ///         (malformed file) and concrete-ctor (bad geometry) exceptions.
    static void load(RocketModel& rocket, MotorModelDatabase& motors, const std::string& filename);
};

} // namespace model

#endif // MODEL_DESIGNSERIALIZER_H
