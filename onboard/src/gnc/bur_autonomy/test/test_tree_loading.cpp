// A-3: every top-level behavior_trees/*.xml must load against the REAL
// factory (registerLeaves via buildMissionTree, exactly what the manager uses).
//
// Last year's comp_tree.xml referenced four node types nobody had registered,
// and nothing caught it. Here an unregistered node, or an XML attribute that is
// not a declared port, fails this test (and so `colcon test`).
// behavior_trees/reference/ is excluded on purpose: those trees are kept for
// the record and are known not to load.

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "bur_autonomy/register.hpp"

namespace fs = std::filesystem;
using bur_autonomy::MissionContext;

namespace
{

std::vector<fs::path> topLevelTrees()
{
    std::vector<fs::path> out;
    for (const auto & e : fs::directory_iterator(BUR_TREE_DIR)) {
        if (e.is_regular_file() && e.path().extension() == ".xml") { out.push_back(e.path()); }
    }
    return out;
}

std::string wrap(const std::string & body)
{
    return "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">" + body +
           "</BehaviorTree></root>";
}

}  // namespace

TEST(TreeLoading, EveryTopLevelTreeLoads)
{
    const auto trees = topLevelTrees();
    // fish_tree.xml and fish_tree_horizontal.xml at least; an empty glob must
    // not pass vacuously.
    ASSERT_GE(trees.size(), 2u);
    for (const auto & p : trees) {
        auto ctx = std::make_shared<MissionContext>();
        EXPECT_NO_THROW(bur_autonomy::buildMissionTree(p.string(), ctx)) << p;
    }
}

TEST(TreeLoading, UnregisteredNodeFails)
{
    auto ctx = std::make_shared<MissionContext>();
    EXPECT_ANY_THROW(bur_autonomy::buildMissionTreeFromText(wrap("<BarrelRoll/>"), ctx));
}

TEST(TreeLoading, UndeclaredAttributeFails)
{
    // The XML-side version of last year's gate_target / gate_position bug.
    auto ctx = std::make_shared<MissionContext>();
    EXPECT_ANY_THROW(bur_autonomy::buildMissionTreeFromText(
        wrap(R"(<SwimForward distance_m="1" speed_mps="0.3" sped_mps="0.5"/>)"), ctx));
}

TEST(TreeLoading, MissingRequiredPortFailsAtFirstTick)
{
    // A missing required input loads (BT.CPP cannot know which ports are
    // required) but must fail loudly on the first tick, never act on garbage.
    auto ctx = std::make_shared<MissionContext>();
    auto m = bur_autonomy::buildMissionTreeFromText(wrap(R"(<SwimForward speed_mps="0.3"/>)"), ctx);
    EXPECT_THROW(m.tree.tickOnce(), BT::RuntimeError);
}

TEST(TreeLoading, ReferenceTreesReallyDoNotLoad)
{
    // Documents WHY reference/ is excluded; if one of these starts loading,
    // it can move back to the top level and be covered by the test above.
    const fs::path comp = fs::path(BUR_TREE_DIR) / "reference" / "comp_tree.xml";
    ASSERT_TRUE(fs::exists(comp));
    auto ctx = std::make_shared<MissionContext>();
    EXPECT_ANY_THROW(bur_autonomy::buildMissionTree(comp.string(), ctx));
}
