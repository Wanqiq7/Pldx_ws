#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

namespace Sentry_BT {

class SentryBTManager
{
public:
  SentryBTManager() = default;
  ~SentryBTManager() = default;

  bool initialize(std::shared_ptr<BT::Blackboard> blackboard,
    const std::string & tree_root_dir,
    unsigned groot2_port = 1667);

  // Main loop framework: tick main tree exactly once at a fixed rate.
  void run(double frequency_hz = 10.0);

  // Exposed hook for external schedulers that want exact single-tick control.
  void tickMainExactlyOnce();
  void tickStanceExactlyOnce();
  void tickTacticalExactlyOnce();

private:
  void registerNodes();
  bool loadTrees(const std::shared_ptr<BT::Blackboard> & blackboard);

  BT::BehaviorTreeFactory factory_;
  BT::Tree resource_tree_;
  BT::Tree nav_tree_;
  BT::Tree gimbal_tree_;
  BT::Tree stance_tree_;
  BT::Tree tactical_tree_;

  // Default main tree can be adjusted later if needed.
  BT::Tree * main_tree_ = nullptr;
  std::unique_ptr<BT::Groot2Publisher> groot2_publisher_;
  std::string tree_root_dir_;
};

}  // namespace Sentry_BT
