/// \cond
#include <algorithm>
#include <string>
#include <utility>
#include <vector>
/// \endcond

#include <gtest/gtest.h>

#include "sim/Integrator.h"
#include "utils/math/MathTypes.h"

namespace
{

// A trivial well-defined ODE (x' = v, v' = 0) so step() has something to integrate.
std::pair<Vector3, Vector3> trivialOdes(double /*t*/, Vector3& /*state*/, Vector3& rate)
{
   return std::make_pair(rate, Vector3(0.0, 0.0, 0.0));
}

bool listContains(const std::vector<std::string>& models, const std::string& name)
{
   return std::find(models.begin(), models.end(), name) != models.end();
}

} // namespace

// An unknown name must not mutate state: the model list keeps exactly its two real keys (no "None",
// no echo of the bad name), and the previously-selected solver stays usable -- step() neither
// null-derefs (the old "None" bug) nor otherwise fails.
TEST(IntegratorTest, UnknownModelIsLoggedNoOp)
{
   sim::Integrator integ;
   integ.setIntegratorFunction(trivialOdes);
   integ.setIntegratorModel("Runge-Kutta 4th Order"); // rebuild the active solver with the ODE fn
   integ.setTimeStep(0.01);

   integ.setIntegratorModel("Bogus"); // unknown -> logged no-op, RK4 retained

   const std::vector<std::string> models = integ.getAvailableIntegratorModels();
   EXPECT_EQ(models.size(), 2u);
   EXPECT_FALSE(listContains(models, "None"));
   EXPECT_FALSE(listContains(models, "Bogus"));
   EXPECT_TRUE(listContains(models, "Runge-Kutta 4th Order"));
   EXPECT_TRUE(listContains(models, "Runge-Kutta-Fehlberg"));

   // The RK4 solver set up before the bad call is still active and fully usable.
   Vector3 state(0.0, 0.0, 0.0);
   Vector3 rate(0.0, 0.0, 1.0);
   EXPECT_NO_THROW({ (void)integ.step(0.0, state, rate); });
}

// Both real models remain selectable and switching between them keeps the list at two keys.
TEST(IntegratorTest, ValidModelsRemainSelectable)
{
   sim::Integrator integ;
   integ.setIntegratorFunction(trivialOdes);

   Vector3 state(0.0, 0.0, 0.0);
   Vector3 rate(0.0, 0.0, 1.0);

   integ.setIntegratorModel("Runge-Kutta-Fehlberg");
   integ.setTimeStep(0.01);
   EXPECT_EQ(integ.getAvailableIntegratorModels().size(), 2u);
   EXPECT_NO_THROW({ (void)integ.step(0.0, state, rate); });

   integ.setIntegratorModel("Runge-Kutta 4th Order");
   integ.setTimeStep(0.01); // the freshly (re)created solver needs its own step size
   EXPECT_EQ(integ.getAvailableIntegratorModels().size(), 2u);
   EXPECT_NO_THROW({ (void)integ.step(0.0, state, rate); });
}
