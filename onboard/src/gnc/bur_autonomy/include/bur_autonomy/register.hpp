#ifndef BUR_AUTONOMY_REGISTER_HPP
#define BUR_AUTONOMY_REGISTER_HPP

// The ONE registration point for the leaves, and the ONE way to build a
// mission tree. The manager node and every test go through these, so a tree
// that loads in a test is a tree that loads on the vehicle.

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

#include "bur_autonomy/leaves.hpp"

namespace bur_autonomy
{

void registerLeaves(BT::BehaviorTreeFactory & factory, const ContextPtr & ctx);

// A tree together with the factory that built it, so the factory outlives it.
struct MissionTree
{
    std::unique_ptr<BT::BehaviorTreeFactory> factory;
    BT::Tree tree;
};

// Load `xml_path` and instantiate its "MainTree". Throws (BT::RuntimeError or
// std::exception) on an unregistered node, an undeclared attribute, a missing
// file or a missing MainTree.
MissionTree buildMissionTree(const std::string & xml_path, const ContextPtr & ctx);

// Same, from XML text (tests).
MissionTree buildMissionTreeFromText(const std::string & xml_text, const ContextPtr & ctx);

}  // namespace bur_autonomy

#endif
