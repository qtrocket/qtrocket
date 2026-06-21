#include <gtest/gtest.h>

#include <memory>

#include "model/RocketModel.h"
#include "model/parts/Parts.h"

namespace
{
using model::RocketModel;
using model::part::Part;

std::shared_ptr<model::part::BodyTube> bodyTube(const std::string& name)
{
   return std::make_shared<model::part::BodyTube>(name, 0.0, 0.019, 0.20, 680.0);
}
std::shared_ptr<model::part::FinSet> finSet(const std::string& name)
{
   return std::make_shared<model::part::FinSet>(name, 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0);
}
} // namespace

TEST(RocketModelFacadeTest, GetTopPartReturnsTheBootPlaceholder)
{
   RocketModel r;
   ASSERT_NE(r.getTopPart(), nullptr);
   EXPECT_EQ(r.getTopPart()->typeName(), "HollowSphere");
   EXPECT_EQ(r.getTopPart()->getName(), "Body");
}

TEST(RocketModelFacadeTest, SetRootReplacesTreeAndResetsReferenceAreaOverride)
{
   RocketModel r;
   r.setReferenceArea(0.05); // manual override on
   EXPECT_TRUE(r.isReferenceAreaOverridden());

   auto body = bodyTube("Tube");
   const double bodyMass = body->getMass(0.0);
   r.setRoot(body);

   EXPECT_FALSE(r.isReferenceAreaOverridden());  // a new airframe drops the stale manual override
   EXPECT_EQ(r.getTopPart()->typeName(), "BodyTube");
   EXPECT_DOUBLE_EQ(r.getMass(0.0), bodyMass);   // composite reflects the new root
}

TEST(RocketModelFacadeTest, SetRootIgnoresNullAndKeepsTreeValid)
{
   RocketModel r;
   r.setRoot(nullptr);                      // logged no-op
   ASSERT_NE(r.getTopPart(), nullptr);
   EXPECT_EQ(r.getTopPart()->typeName(), "HollowSphere");
}

TEST(RocketModelFacadeTest, ClearDesignRestoresThePlaceholder)
{
   RocketModel r;
   r.setRoot(bodyTube("Tube"));
   ASSERT_EQ(r.getTopPart()->typeName(), "BodyTube");

   r.clearDesign();
   EXPECT_EQ(r.getTopPart()->typeName(), "HollowSphere");
   EXPECT_EQ(r.getTopPart()->getName(), "Body");
}

TEST(RocketModelFacadeTest, AddPartAttachesUnderParentAndReportsSuccessOrFailure)
{
   RocketModel r;
   r.setRoot(bodyTube("Body"));
   const Part::Id rootId = r.getTopPart()->getId();
   const double rootMass = r.getMass(0.0);

   auto fins = finSet("Fins");
   const double finsMass = fins->getMass(0.0);
   EXPECT_TRUE(r.addPart(rootId, fins, Vector3{0.0, 0.0, -0.10}));
   EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass + finsMass); // composite grew by the fins

   // Bad parent id -> false, and the tree is unchanged.
   EXPECT_FALSE(r.addPart(999999u, bodyTube("Orphan"), Vector3::Zero()));
   EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass + finsMass);
}

TEST(RocketModelFacadeTest, RemovePartRefusesRootAndDetachesChildren)
{
   RocketModel r;
   r.setRoot(bodyTube("Body"));
   const Part::Id rootId = r.getTopPart()->getId();
   const double rootMass = r.getMass(0.0);

   auto fins = finSet("Fins");
   const Part::Id finsId = fins->getId();
   const double finsMass = fins->getMass(0.0);
   ASSERT_TRUE(r.addPart(rootId, fins, Vector3{0.0, 0.0, -0.10}));

   // The root cannot be removed via removePart; the tree stays intact.
   EXPECT_EQ(r.removePart(rootId), nullptr);
   EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass + finsMass);

   // Removing the fins returns the sub-tree and shrinks the composite back to the bare body.
   auto detached = r.removePart(finsId);
   ASSERT_NE(detached, nullptr);
   EXPECT_EQ(detached->getId(), finsId);
   EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass);

   // A genuinely absent id is a no-op returning nullptr (the tree is unchanged).
   EXPECT_EQ(r.removePart(123456789u), nullptr);
   EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass);
}

TEST(RocketModelFacadeTest, AddPartAttachesUnderANonRootDescendant)
{
   RocketModel r;
   r.setRoot(bodyTube("Body"));
   const Part::Id rootId = r.getTopPart()->getId();

   auto mid = bodyTube("Mid");
   const Part::Id midId = mid->getId();
   ASSERT_TRUE(r.addPart(rootId, mid, Vector3{0.0, 0.0, -0.20})); // mid under the root

   auto leaf = finSet("Fins");
   const Part::Id leafId = leaf->getId();
   const double massBefore = r.getMass(0.0);
   const double leafMass = leaf->getMass(0.0);
   EXPECT_TRUE(r.addPart(midId, leaf, Vector3{0.0, 0.0, -0.10})); // leaf under the descendant `mid`
   EXPECT_DOUBLE_EQ(r.getMass(0.0), massBefore + leafMass);       // composite includes the deep child
   ASSERT_NE(r.findPart(leafId), nullptr);                        // and findPart reaches it
   EXPECT_EQ(r.findPart(leafId)->getName(), "Fins");
}

TEST(RocketModelFacadeTest, RemovePartOfTheMotorBranchDropsTheMotor)
{
   // addPart of a Motor sub-tree must re-resolve motorPart (so isMotorSet sees it), and removePart of
   // that branch must re-resolve back to null -- the removePart side of Guardrail 6.
   RocketModel r;
   r.setRoot(bodyTube("Body"));
   const Part::Id rootId = r.getTopPart()->getId();

   auto motor = std::make_shared<model::part::Motor>("M", model::MotorModel{});
   const Part::Id motorId = motor->getId();
   ASSERT_TRUE(r.addPart(rootId, motor, Vector3::Zero()));
   EXPECT_TRUE(r.isMotorSet()); // addPart re-resolved and found the attached motor

   auto detached = r.removePart(motorId);
   ASSERT_NE(detached, nullptr);
   EXPECT_FALSE(r.isMotorSet());            // removePart re-resolved -> no motor remains
   EXPECT_DOUBLE_EQ(r.getThrust(1.0), 0.0); // safe, zero thrust
}

TEST(RocketModelFacadeTest, SetRootReresolvesMotorPartSoThrustNeverDangles)
{
   // Guardrail 6: the borrowed motorPart belongs to whatever tree is installed. Replacing the tree
   // must re-resolve it (to the new tree's motor, or null) so getThrust() can never read freed memory
   // and isMotorSet() never reports a stale pointer.
   RocketModel r;
   r.setMotorModel(model::MotorModel{}); // attaches a Motor child to the placeholder tree
   EXPECT_TRUE(r.isMotorSet());

   // Install a NEW tree that itself contains a Motor node -> motorPart must re-borrow it (the old
   // tree, and its Motor, are freed when topPart is replaced).
   auto root2 = bodyTube("Body2");
   root2->addChildPart(std::make_shared<model::part::Motor>("M2", model::MotorModel{}), Vector3::Zero());
   r.setRoot(root2);
   EXPECT_TRUE(r.isMotorSet());             // re-borrowed from the new tree, not the freed old one
   EXPECT_NO_THROW((void)r.getThrust(1.0)); // safe: no dangling pointer

   // Install a motor-LESS tree -> motorPart must drop to null (a stale non-null would be the bug).
   r.setRoot(bodyTube("Body3"));
   EXPECT_FALSE(r.isMotorSet());
   EXPECT_DOUBLE_EQ(r.getThrust(1.0), 0.0); // no motor -> zero thrust, no crash

   // setMotorModel after a motor-less setRoot re-attaches cleanly to the current tree.
   r.setMotorModel(model::MotorModel{});
   EXPECT_TRUE(r.isMotorSet());
}
