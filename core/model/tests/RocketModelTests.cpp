#include <gtest/gtest.h>

#include <memory>
#include <utility>

#include "model/PartsModel.h"
#include "model/RocketModel.h"
#include "model/parts/Parts.h"

namespace
{
using model::PartNode;
using model::PartsModel;
using model::RocketModel;
using model::part::Part;

std::unique_ptr<model::part::BodyTube> bodyTube(const std::string& name)
{
    return std::make_unique<model::part::BodyTube>(name, 0.0, 0.019, 0.20, 680.0);
}
std::unique_ptr<model::part::FinSet> finSet(const std::string& name)
{
    return std::make_unique<model::part::FinSet>(name, 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0);
}
} // namespace

TEST(RocketModelFacadeTest, InstallDesignReplacesTreeAndResetsReferenceAreaOverride)
{
    RocketModel r;
    r.setReferenceArea(0.05); // manual override on
    EXPECT_TRUE(r.isReferenceAreaOverridden());

    auto body = bodyTube("Tube");
    const double bodyMass = body->getMass(0.0);
    r.installDesign(PartNode::make(std::move(body)));

    EXPECT_FALSE(r.isReferenceAreaOverridden());  // a new airframe drops the stale manual override
    ASSERT_NE(r.parts().root(), nullptr);
    EXPECT_EQ(r.parts().root()->part().typeName(), "BodyTube");
    EXPECT_DOUBLE_EQ(r.getMass(0.0), bodyMass);   // composite reflects the new root
}

TEST(RocketModelFacadeTest, ClearDesignClearsTreeAndMotorAndFiresCallback)
{
    RocketModel r;
    r.installDesign(PartNode::make(bodyTube("Tube")));
    r.setMotorModel(model::MotorModel{});
    ASSERT_TRUE(r.isMotorSet());

    int fired = 0;
    r.setStructureChangedCallback([&fired]() { ++fired; });

    r.clearDesign();
    EXPECT_FALSE(r.parts().hasDesign());          // "no design" is a real state
    EXPECT_EQ(r.parts().root(), nullptr);
    EXPECT_FALSE(r.isMotorSet());                 // the motor borrow cannot outlive the tree
    EXPECT_DOUBLE_EQ(r.getThrust(1.0), 0.0);      // safe, zero thrust
    EXPECT_GT(fired, 0);                          // clearing is a structural change the GUI must see
}

TEST(RocketModelFacadeTest, AttachAddsUnderParentAndReportsTypedErrors)
{
    RocketModel r;
    r.installDesign(PartNode::make(bodyTube("Body")));
    const Part::Id rootId = r.parts().root()->id();
    const double rootMass = r.getMass(0.0);

    auto fins = finSet("Fins");
    const Part::Id finsId = fins->getId();
    const double finsMass = fins->getMass(0.0);
    const auto attached = r.parts().attach(rootId, std::move(fins), model::part::abut(-0.10));
    ASSERT_TRUE(attached.has_value());
    EXPECT_EQ(*attached, finsId);                          // the moved-in part keeps its id
    EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass + finsMass); // composite grew by the fins

    // Bad parent id -> typed error, and the tree is unchanged.
    const auto orphan = r.parts().attach(999999U, bodyTube("Orphan"), model::part::abut());
    ASSERT_FALSE(orphan.has_value());
    EXPECT_EQ(orphan.error(), PartsModel::AttachError::NoSuchParent);
    EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass + finsMass);

    // A null part is refused outright.
    const auto null = r.parts().attach(rootId, nullptr, model::part::abut());
    ASSERT_FALSE(null.has_value());
    EXPECT_EQ(null.error(), PartsModel::AttachError::NullPart);
    EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass + finsMass);
}

TEST(RocketModelFacadeTest, DetachRefusesRootAndReturnsTheSubtree)
{
    RocketModel r;
    r.installDesign(PartNode::make(bodyTube("Body")));
    const Part::Id rootId = r.parts().root()->id();
    const double rootMass = r.getMass(0.0);

    auto fins = finSet("Fins");
    const Part::Id finsId = fins->getId();
    const double finsMass = fins->getMass(0.0);
    ASSERT_TRUE(r.parts().attach(rootId, std::move(fins), model::part::abut(-0.10)).has_value());

    // The root cannot be detached; the tree stays intact.
    const auto rootDetach = r.parts().detach(rootId);
    ASSERT_FALSE(rootDetach.has_value());
    EXPECT_EQ(rootDetach.error(), PartsModel::DetachError::IsRoot);
    EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass + finsMass);

    // Detaching the fins returns the owned sub-tree and shrinks the composite back to the bare body.
    auto detached = r.parts().detach(finsId);
    ASSERT_TRUE(detached.has_value());
    ASSERT_NE(*detached, nullptr);
    EXPECT_EQ((*detached)->id(), finsId);
    EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass);

    // A genuinely absent id is a typed miss; the tree is unchanged.
    const auto absent = r.parts().detach(123456789U);
    ASSERT_FALSE(absent.has_value());
    EXPECT_EQ(absent.error(), PartsModel::DetachError::NoSuchId);
    EXPECT_DOUBLE_EQ(r.getMass(0.0), rootMass);
}

TEST(RocketModelFacadeTest, AttachUnderANonRootDescendant)
{
    RocketModel r;
    r.installDesign(PartNode::make(bodyTube("Body")));
    const Part::Id rootId = r.parts().root()->id();

    auto mid = bodyTube("Mid");
    const Part::Id midId = mid->getId();
    ASSERT_TRUE(r.parts().attach(rootId, std::move(mid), model::part::abut(-0.20)).has_value());

    auto leaf = finSet("Fins");
    const Part::Id leafId = leaf->getId();
    const double massBefore = r.getMass(0.0);
    const double leafMass = leaf->getMass(0.0);
    EXPECT_TRUE(r.parts().attach(midId, std::move(leaf), model::part::abut(-0.10)).has_value());
    EXPECT_DOUBLE_EQ(r.getMass(0.0), massBefore + leafMass); // composite includes the deep child
    ASSERT_NE(r.parts().find(leafId), nullptr);              // and find() reaches it
    EXPECT_EQ(r.parts().find(leafId)->part().getName(), "Fins");
}

TEST(RocketModelFacadeTest, DetachOfTheMotorNodeDropsTheMotor)
{
    // Attaching a Motor part must re-resolve the motor borrow (so isMotorSet sees it), a second
    // motor is refused, and detaching the motor's node resolves the borrow back to null.
    RocketModel r;
    r.installDesign(PartNode::make(bodyTube("Body")));
    const Part::Id rootId = r.parts().root()->id();

    auto motor = std::make_unique<model::part::Motor>("M", model::MotorModel{});
    const Part::Id motorId = motor->getId();
    ASSERT_TRUE(r.parts().attach(rootId, std::move(motor), model::part::abut()).has_value());
    EXPECT_TRUE(r.isMotorSet());

    const auto second =
        r.parts().attach(rootId, std::make_unique<model::part::Motor>("M2", model::MotorModel{}),
                         model::part::abut());
    ASSERT_FALSE(second.has_value());
    EXPECT_EQ(second.error(), PartsModel::AttachError::DuplicateMotor);

    auto detached = r.parts().detach(motorId);
    ASSERT_TRUE(detached.has_value());
    EXPECT_FALSE(r.isMotorSet());            // no motor remains after the borrow re-resolves
    EXPECT_DOUBLE_EQ(r.getThrust(1.0), 0.0); // safe, zero thrust
}

TEST(RocketModelFacadeTest, InstallDesignReresolvesMotorBorrowSoThrustNeverDangles)
{
    // The borrowed motor pointer belongs to whatever tree is installed. Replacing the tree must
    // re-resolve it (to the new tree's motor, or null) so getThrust() can never read freed memory
    // and isMotorSet() never reports a stale pointer.
    RocketModel r;
    r.installDesign(PartNode::make(bodyTube("Body")));
    r.setMotorModel(model::MotorModel{}); // attaches a Motor node to the installed tree
    EXPECT_TRUE(r.isMotorSet());

    // Install a NEW tree that itself contains a Motor node -> the borrow must re-resolve to it (the
    // old tree, and its Motor, are freed by the replace).
    auto root2 = PartNode::make(bodyTube("Body2"));
    root2->addChild(PartNode::make(std::make_unique<model::part::Motor>("M2", model::MotorModel{})),
                    model::part::abut());
    r.installDesign(std::move(root2));
    EXPECT_TRUE(r.isMotorSet());             // borrowed from the new tree, not the freed old one
    EXPECT_NO_THROW((void)r.getThrust(1.0)); // safe: no dangling pointer

    // Install a motor-LESS tree -> the borrow must drop to null (a stale non-null would be the bug).
    r.installDesign(PartNode::make(bodyTube("Body3")));
    EXPECT_FALSE(r.isMotorSet());
    EXPECT_DOUBLE_EQ(r.getThrust(1.0), 0.0); // no motor -> zero thrust, no crash

    // setMotorModel after a motor-less install re-attaches cleanly to the current tree.
    r.setMotorModel(model::MotorModel{});
    EXPECT_TRUE(r.isMotorSet());
}
