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
 * @brief Minimal concrete Part for unit tests: a bare composition node that carries an explicitly
 *        supplied mass and (per-unit-mass) inertia tensor, with no derived geometry.
 *
 * Part is abstract -- typeName() and cloneShallow() are pure -- so a test that needs a generic node
 * (a point mass, a synthetic tube, an anonymous tree node) instantiates this stand-in instead of the
 * base. The composition math these tests exercise (mass aggregation, CM, parallel-axis inertia) is
 * geometry-agnostic, so a node with hand-picked mass properties is exactly the right tool and there is
 * no need to borrow a real geometry type.
 */
class TestPart : public Part
{
public:
   using Part::Part; ///< inherit the (name, I, m, centerMass) constructor verbatim

   std::string typeName() const override { return "TestPart"; }

protected:
   /// @brief Uses Part's protected copy ctor (own mass properties, fresh id, no children), so clone()
   ///        reproduces a TestPart.
   TestPart(const TestPart&) = default;

   std::shared_ptr<Part> cloneShallow() const override
   {
      return std::shared_ptr<Part>(new TestPart(*this));
   }
};

} // namespace model::part

#endif // MODEL_TESTS_TESTPART_H
