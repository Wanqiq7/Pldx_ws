#pragma once

#include <behaviortree_cpp/action_node.h>

#include "bt_manager/utils/nav_zone.hpp"

namespace Sentry_BT {
class ChangeTacticalAction : public BT::SyncActionNode
{
public:
  ChangeTacticalAction(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

class SetTacticalMode : public BT::SyncActionNode
{
public:
  SetTacticalMode(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace Sentry_BT
