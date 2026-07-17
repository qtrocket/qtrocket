// Contract tests for RocketPartModel's granular event consumption: QAbstractItemModelTester
// validates the QAbstractItemModel protocol while a full edit sequence streams through the
// aboutTo/did events, and the surgical guarantees (persistent indexes survive unrelated edits,
// leaf edits emit dataChanged not resets, only a design install resets) are pinned directly.

/// \cond
#include <memory>
#include <utility>

#include <QAbstractItemModelTester>
#include <QCoreApplication>
#include <QPersistentModelIndex>
#include <QSignalSpy>

#include <gtest/gtest.h>
/// \endcond

#include "RocketTreeView.h"
#include "model/PartsModel.h"
#include "model/RocketModel.h"
#include "model/parts/Parts.h"

namespace
{

std::unique_ptr<model::part::BodyTube> bodyTube(const std::string& name)
{
    return std::make_unique<model::part::BodyTube>(name, 0.0, 0.019, 0.20, 680.0);
}

/// RocketModel + RocketPartModel wired the way RocketTreeView wires them, minus the widget, plus a
/// continuously-attached model tester that qFatals on any contract violation.
class RocketPartModelTests : public ::testing::Test
{
protected:
    void SetUp() override
    {
        partModel.setParts(&rocket.parts());
        rocket.setPartsEventCallback([this](const model::PartsModel::Event& e, bool before)
        { partModel.onPartsEvent(e, before); });
        tester = std::make_unique<QAbstractItemModelTester>(
            &partModel, QAbstractItemModelTester::FailureReportingMode::Fatal);
    }

    void TearDown() override { rocket.setPartsEventCallback({}); }

    model::RocketModel rocket;
    RocketPartModel partModel;
    std::unique_ptr<QAbstractItemModelTester> tester;
};

} // namespace

TEST_F(RocketPartModelTests, ContractHoldsThroughFullEditSequence)
{
    // Every mutation streams through the tester; a protocol violation aborts the test.
    rocket.installDesign(model::PartNode::make(
        std::make_unique<model::part::ConicalNoseCone>("Nose", 0.019, 0.10, 0.0, 2700.0, true)));
    const auto noseId = rocket.parts().root()->id();
    ASSERT_EQ(partModel.rowCount({}), 1);

    const auto bodyId = rocket.parts().attach(noseId, bodyTube("Body"));
    ASSERT_TRUE(bodyId.has_value());
    const auto finsId = rocket.parts().attach(
        *bodyId, std::make_unique<model::part::FinSet>("Fins", 3, 0.08, 0.04, 0.05, 0.03, 0.003,
                                                                    0.019, 600.0));
    ASSERT_TRUE(finsId.has_value());

    const QModelIndex noseIdx = partModel.index(0, 0, {});
    ASSERT_EQ(partModel.rowCount(noseIdx), 1);
    const QModelIndex bodyIdx = partModel.index(0, 0, noseIdx);
    EXPECT_EQ(partModel.data(bodyIdx, Qt::DisplayRole).toString(), QStringLiteral("Body"));
    EXPECT_EQ(partModel.rowCount(bodyIdx), 1);

    EXPECT_TRUE(rocket.parts().setPartMass(*bodyId, 0.123));
    EXPECT_TRUE(rocket.parts().setLink(*finsId, model::part::seatOnWall(0.1)));
    ASSERT_TRUE(rocket.parts().detach(*finsId).has_value());
    EXPECT_EQ(partModel.rowCount(bodyIdx), 0);

    rocket.clearDesign();
    EXPECT_EQ(partModel.rowCount({}), 0);

    rocket.installDesign(model::PartNode::make(bodyTube("Fresh")));
    EXPECT_EQ(partModel.rowCount({}), 1);
}

TEST_F(RocketPartModelTests, PersistentIndexSurvivesUnrelatedEditsAndDiesWithItsNode)
{
    rocket.installDesign(model::PartNode::make(bodyTube("Root")));
    const auto rootId = rocket.parts().root()->id();
    const auto bodyId = rocket.parts().attach(rootId, bodyTube("Body"));
    ASSERT_TRUE(bodyId.has_value());

    const QPersistentModelIndex body(partModel.index(0, 0, partModel.index(0, 0, {})));
    ASSERT_TRUE(body.isValid());
    const auto heldId = static_cast<model::part::Part::Id>(body.internalId());
    EXPECT_EQ(heldId, *bodyId);

    // unrelated edits: a sibling attach and a leaf edit; the held index stays valid and stable
    ASSERT_TRUE(rocket.parts().attach(rootId, bodyTube("Sibling")).has_value());
    ASSERT_TRUE(rocket.parts().setPartMass(rootId, 0.5));
    EXPECT_TRUE(body.isValid());
    EXPECT_EQ(static_cast<model::part::Part::Id>(body.internalId()), *bodyId);
    EXPECT_EQ(body.data().toString(), QStringLiteral("Body"));

    // removing the node invalidates exactly its index; the id can never alias a live row
    ASSERT_TRUE(rocket.parts().detach(*bodyId).has_value());
    EXPECT_FALSE(body.isValid());
}

TEST_F(RocketPartModelTests, LeafEditEmitsDataChangedNotReset)
{
    rocket.installDesign(model::PartNode::make(bodyTube("Root")));
    const auto rootId = rocket.parts().root()->id();

    QSignalSpy dataChanged(&partModel, &QAbstractItemModel::dataChanged);
    QSignalSpy resets(&partModel, &QAbstractItemModel::modelReset);

    ASSERT_TRUE(rocket.parts().setPartMass(rootId, 0.321));

    ASSERT_EQ(dataChanged.count(), 1);
    const auto args = dataChanged.takeFirst();
    const auto left = args.at(0).toModelIndex();
    EXPECT_EQ(static_cast<model::part::Part::Id>(left.internalId()), rootId);
    EXPECT_EQ(partModel.data(partModel.index(0, RocketPartModel::Mass, {}), Qt::DisplayRole).toString(),
                 QStringLiteral("0.3210"));
    EXPECT_EQ(resets.count(), 0);
}

TEST_F(RocketPartModelTests, OnlyADesignInstallResets)
{
    rocket.installDesign(model::PartNode::make(bodyTube("Root")));
    const auto rootId = rocket.parts().root()->id();

    QSignalSpy resets(&partModel, &QAbstractItemModel::modelReset);
    QSignalSpy inserts(&partModel, &QAbstractItemModel::rowsInserted);
    QSignalSpy removes(&partModel, &QAbstractItemModel::rowsRemoved);

    const auto bodyId = rocket.parts().attach(rootId, bodyTube("Body"));
    ASSERT_TRUE(bodyId.has_value());
    ASSERT_TRUE(rocket.parts().detach(*bodyId).has_value());

    EXPECT_EQ(inserts.count(), 1);
    EXPECT_EQ(removes.count(), 1);
    EXPECT_EQ(resets.count(), 0);

    rocket.installDesign(model::PartNode::make(bodyTube("Fresh")));
    EXPECT_EQ(resets.count(), 1);
}

int main(int argc, char** argv)
{
    // QCoreApplication, not QApplication: the model under test is widget-free, so the suite runs
    // headless with no platform plugin.
    QCoreApplication app(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
