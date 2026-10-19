#include "minco_core/minco_planner.hpp"

#include <gtest/gtest.h>

namespace minco_planner {
namespace {

// Matching empty frames exercise the goal interface without TF or a running map.
geometry_msgs::msg::PoseStamped makeGoal()
{
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 2.0;
  goal.pose.position.y = 3.0;
  goal.pose.orientation.w = 1.0;
  return goal;
}

TEST(GoalDeduplication, ReturnsPathWithoutResubmittingConsumedGoal)
{
  MincoPlanner planner;
  auto start = makeGoal();
  auto goal = makeGoal();
  geometry_msgs::msg::PoseStamped consumed;

  ASSERT_EQ(planner.createPlan(start, goal, {}).poses.size(), 2U);
  ASSERT_TRUE(planner.consumePendingGoal(consumed));

  start.pose.position.x = 1.0;
  goal.header.stamp.sec = 42;
  const auto path = planner.createPlan(start, goal, {});
  ASSERT_EQ(path.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(path.poses.front().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, 2.0);
  EXPECT_FALSE(planner.consumePendingGoal(consumed));
}

TEST(GoalDeduplication, KeepsLatestDistinctPendingGoal)
{
  MincoPlanner planner;
  auto goal = makeGoal();
  planner.createPlan(goal, goal, {});
  planner.createPlan(goal, goal, {});
  goal.pose.position.x += 0.001;
  planner.createPlan(goal, goal, {});

  geometry_msgs::msg::PoseStamped consumed;
  ASSERT_TRUE(planner.consumePendingGoal(consumed));
  EXPECT_DOUBLE_EQ(consumed.pose.position.x, goal.pose.position.x);
  EXPECT_FALSE(planner.consumePendingGoal(consumed));
}

TEST(GoalDeduplication, UsesPositionForGoalEquivalence)
{
  MincoPlanner planner;
  auto goal = makeGoal();
  geometry_msgs::msg::PoseStamped consumed;
  planner.createPlan(goal, goal, {});
  ASSERT_TRUE(planner.consumePendingGoal(consumed));

  goal.pose.orientation.w = -1.0;
  planner.createPlan(goal, goal, {});
  EXPECT_FALSE(planner.consumePendingGoal(consumed));

  goal.pose.orientation.z = 1.0;
  goal.pose.orientation.w = 0.0;
  const auto path = planner.createPlan(goal, goal, {});
  ASSERT_EQ(path.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.orientation.z, 1.0);
  EXPECT_FALSE(planner.consumePendingGoal(consumed));

  goal.pose.position.y += 0.001;
  planner.createPlan(goal, goal, {});
  ASSERT_TRUE(planner.consumePendingGoal(consumed));
  EXPECT_DOUBLE_EQ(consumed.pose.orientation.z, 1.0);
}

TEST(GoalDeduplication, UsesEuclideanPositionDistance)
{
  MincoPlanner planner;
  auto goal = makeGoal();
  geometry_msgs::msg::PoseStamped consumed;
  planner.createPlan(goal, goal, {});
  ASSERT_TRUE(planner.consumePendingGoal(consumed));

  goal.pose.position.x += 0.75e-9;
  goal.pose.position.y += 0.75e-9;
  planner.createPlan(goal, goal, {});
  EXPECT_TRUE(planner.consumePendingGoal(consumed));

  goal.pose.position.z += 0.001;
  planner.createPlan(goal, goal, {});
  EXPECT_TRUE(planner.consumePendingGoal(consumed));
}

TEST(GoalDeduplication, FilteredNoiseDoesNotMoveAcceptedReference)
{
  MincoPlanner planner;
  auto goal = makeGoal();
  geometry_msgs::msg::PoseStamped consumed;
  planner.createPlan(goal, goal, {});
  ASSERT_TRUE(planner.consumePendingGoal(consumed));

  goal.pose.position.x += 0.75e-9;
  planner.createPlan(goal, goal, {});
  EXPECT_FALSE(planner.consumePendingGoal(consumed));
  goal.pose.position.x += 0.75e-9;
  planner.createPlan(goal, goal, {});
  EXPECT_TRUE(planner.consumePendingGoal(consumed));
}

TEST(GoalDeduplication, CancellationAllowsSameGoalAgain)
{
  MincoPlanner planner;
  const auto goal = makeGoal();
  planner.createPlan(goal, goal, {});
  planner.cancelGoal();
  planner.createPlan(goal, goal, {});
  geometry_msgs::msg::PoseStamped consumed;
  EXPECT_TRUE(planner.consumePendingGoal(consumed));
}

TEST(GoalDeduplication, DeactivationDiscardsPendingGoal)
{
  MincoPlanner planner;
  const auto goal = makeGoal();
  planner.createPlan(goal, goal, {});
  planner.deactivate();
  geometry_msgs::msg::PoseStamped consumed;
  EXPECT_FALSE(planner.consumePendingGoal(consumed));
}

TEST(GoalDeduplication, CancelledSubmissionDoesNotAcceptGoal)
{
  MincoPlanner planner;
  const auto goal = makeGoal();
  EXPECT_TRUE(planner
                .createPlan(goal,
                  goal,
                  [] {
                    return true;
                  })
                .poses.empty());
  geometry_msgs::msg::PoseStamped consumed;
  EXPECT_FALSE(planner.consumePendingGoal(consumed));
}

TEST(GoalDeduplication, FailedNormalizationDoesNotLosePendingGoalOnRetry)
{
  MincoPlanner planner;
  const auto goal = makeGoal();
  planner.createPlan(goal, goal, {});
  auto invalid_start = goal;
  invalid_start.header.frame_id = "unavailable_frame";
  EXPECT_TRUE(planner.createPlan(invalid_start, goal, {}).poses.empty());
  planner.createPlan(goal, goal, {});
  geometry_msgs::msg::PoseStamped consumed;
  EXPECT_TRUE(planner.consumePendingGoal(consumed));
}

}  // namespace
}  // namespace minco_planner
