#ifndef MODEL_TESTS_TESTPART_H
#define MODEL_TESTS_TESTPART_H

/// \cond
// C++ headers
#include <memory>
#include <string>
/// \endcond

// qtrocket headers
#include "model/parts/Part.h"

namespace model::part
{

/**
 * @brief Minimal concrete Part for unit tests: a bare leaf that carries an explicitly supplied mass
 *        and (per-unit-mass) inertia tensor, with no derived geometry.
 *
 * Part is abstract -- typeName() and clone() are pure -- so a test that needs a generic part (a point
 * mass, an anonymous tree leaf) instantiates this stand-in instead of the base. The composite math
 * these tests exercise (mass aggregation, CM, parallel-axis inertia) is geometry-agnostic, so a part
 * with hand-picked mass properties is exactly the right tool and there is no need to borrow a real
 * geometry type.
 */
class TestPart : public Part
{
public:
    using Part::Part; ///< inherit the (name, I, m, centerMass) constructor verbatim

    std::string typeName() const override { return "TestPart"; }

    std::unique_ptr<Part> clone() const override
    {
        return std::unique_ptr<Part>(new TestPart(*this));
    }

protected:
    /// Uses Part's protected copy ctor: same mass properties, fresh id.
    TestPart(const TestPart&) = default;
};

} // namespace model::part

#endif // MODEL_TESTS_TESTPART_H
