#include "minco_core/components/local_path_processor.hpp"
#include "minco_core/planning_deadline.hpp"

#include <algorithm>
#include <limits>

namespace minco_planner {

namespace {

bool isLineFree(const std::shared_ptr<rog_map::MapQueryInterface> & map,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2)
{
  if (!map || map->resolution() <= 0.0) {
    return true;
  }
  const double dist = (p2 - p1).norm();
  const int steps = static_cast<int>(std::ceil(dist / map->resolution()));
  if (steps <= 0) {
    return true;
  }
  for (int i = 0; i <= steps; ++i) {
    minco_planner::PlanningDeadline::check();
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const Eigen::Vector3d p = p1 + (p2 - p1) * t;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!map->worldToMap(p.x(), p.y(), mx, my) || !map->isFree(mx, my)) {
      return false;
    }
  }
  return true;
}

}  // namespace

void LocalPathProcessor::configure(
  double lookahead_dist, double max_vel, double max_acc, double traj_goal_tolerance, rclcpp::Logger logger)
{
  lookahead_dist_ = lookahead_dist;
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  traj_goal_tolerance_ = traj_goal_tolerance;
  logger_ = logger;
}

void LocalPathProcessor::updateLimits(double max_vel, double max_acc, double traj_goal_tolerance)
{
  max_vel_ = max_vel;
  max_acc_ = max_acc;
  traj_goal_tolerance_ = traj_goal_tolerance;
}

LocalPathSeed LocalPathProcessor::buildSeed(
  const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const PlannerModeContext & mode_context) const
{
  if (global_path.empty()) {
    return {};
  }

  Eigen::Vector3d global_goal(global_path.back().pose.position.x, global_path.back().pose.position.y, 0.0);
  Eigen::Vector3d cur_pos(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);

  return finishSeed(extractLocalPath(global_path, cur_pos), global_goal, mode_context);
}

LocalPathSeed LocalPathProcessor::buildSeedFromHandoff(
  const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
  const geometry_msgs::msg::PoseStamped & current_pose,
  const Eigen::Vector3d & handoff,
  const PlannerModeContext & mode_context) const
{
  if (global_path.size() < 2U || !handoff.allFinite() || !std::isfinite(lookahead_dist_) ||
      lookahead_dist_ <= 0.0) {
    return {};
  }

  struct Projection
  {
    size_t segment{0};
    double ratio{0.0};
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    double distance_sq{std::numeric_limits<double>::infinity()};
    double progress{0.0};
  };
  auto pathPoint = [&global_path](size_t i) {
    const auto & p = global_path[i].pose.position;
    return Eigen::Vector3d(p.x, p.y, 0.0);
  };
  auto project = [&](
                   const Eigen::Vector3d & position, size_t first, double first_ratio, double max_length) {
    Projection best;
    double remaining = max_length;
    double progress = 0.0;
    for (size_t i = first; i + 1U < global_path.size(); ++i) {
      PlanningDeadline::check();
      const Eigen::Vector3d a = pathPoint(i);
      const Eigen::Vector3d delta = pathPoint(i + 1U) - a;
      const double length = delta.head<2>().norm();
      if (length < 1e-9) {
        continue;
      }
      const double lo = i == first ? first_ratio : 0.0;
      const double hi = std::min(1.0, lo + remaining / length);
      const double ratio =
        std::clamp((position - a).head<2>().dot(delta.head<2>()) / (length * length), lo, hi);
      const Eigen::Vector3d projected = a + ratio * delta;
      const double distance_sq = (position - projected).head<2>().squaredNorm();
      if (distance_sq < best.distance_sq) {
        best = {i, ratio, projected, distance_sq, progress + (ratio - lo) * length};
      }
      const double traversed = (hi - lo) * length;
      progress += traversed;
      remaining -= traversed;
      if (remaining <= 1e-9) {
        break;
      }
    }
    return best;
  };

  const Eigen::Vector3d robot(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);
  if (!robot.allFinite()) {
    return {};
  }
  const Projection robot_projection = project(robot, 0U, 0.0, std::numeric_limits<double>::infinity());
  constexpr double kMaxPathDeviation = 1.0;
  if (!std::isfinite(robot_projection.distance_sq) ||
      robot_projection.distance_sq > kMaxPathDeviation * kMaxPathDeviation) {
    return {};
  }

  // A nearby crossing or return leg can be closer than the current branch.
  // Without a progress history for this path version, retain the safe old
  // trajectory rather than arbitrarily choose between equivalent projections.
  constexpr double kAmbiguousDistance = 0.05;
  constexpr double kDistinctProgress = 1.0;
  const double ambiguous_distance = std::sqrt(robot_projection.distance_sq) + kAmbiguousDistance;
  double path_progress = 0.0;
  for (size_t i = 0; i + 1U < global_path.size(); ++i) {
    PlanningDeadline::check();
    const Eigen::Vector3d a = pathPoint(i);
    const Eigen::Vector3d delta = pathPoint(i + 1U) - a;
    const double length = delta.head<2>().norm();
    if (length < 1e-9) {
      continue;
    }
    const double ratio =
      std::clamp((robot - a).head<2>().dot(delta.head<2>()) / (length * length), 0.0, 1.0);
    const double progress = path_progress + ratio * length;
    const double distance = (robot - (a + ratio * delta)).head<2>().norm();
    if (std::abs(progress - robot_projection.progress) > kDistinctProgress &&
        distance <= ambiguous_distance) {
      return {};
    }
    path_progress += length;
  }

  // Search only forward from the robot's progress, and only within this local horizon.
  const Projection anchor =
    project(handoff, robot_projection.segment, robot_projection.ratio, lookahead_dist_);
  const Projection unrestricted = project(
    handoff, robot_projection.segment, robot_projection.ratio, std::numeric_limits<double>::infinity());
  if (!std::isfinite(anchor.distance_sq) || anchor.distance_sq > kMaxPathDeviation * kMaxPathDeviation ||
      unrestricted.distance_sq + 1e-8 < anchor.distance_sq ||
      !isLineFree(mode_context.sparsifyQuery(), handoff, anchor.point) ||
      !isLineFree(mode_context.dynamicQuery(), handoff, anchor.point)) {
    return {};
  }

  std::vector<Eigen::Vector3d> local_path;
  Eigen::Vector3d start = handoff;
  start.z() = 0.0;
  local_path.push_back(start);
  if ((anchor.point - start).head<2>().norm() > 1e-6) {
    local_path.push_back(anchor.point);
  }
  double length = 0.0;
  Eigen::Vector3d previous = anchor.point;
  for (size_t i = anchor.segment + 1U; i < global_path.size(); ++i) {
    PlanningDeadline::check();
    const Eigen::Vector3d next = pathPoint(i);
    const double step = (next - previous).head<2>().norm();
    if (step > 1e-6) {
      local_path.push_back(next);
      length += step;
      previous = next;
    }
    if (length >= lookahead_dist_) {
      break;
    }
  }
  return finishSeed(std::move(local_path), pathPoint(global_path.size() - 1U), mode_context);
}

LocalPathSeed LocalPathProcessor::finishSeed(std::vector<Eigen::Vector3d> path,
  const Eigen::Vector3d & global_goal,
  const PlannerModeContext & mode_context) const
{
  LocalPathSeed seed;
  seed.dense_path = std::move(path);
  const bool clip_required =
    mode_context.mode() == PlannerMode::EXPLORATION || mode_context.clipSeedByRogBoundary();
  const bool clip_ok = clipLocalPathByRogBoundary(seed.dense_path, mode_context);
  if (clip_required && (!clip_ok || seed.dense_path.size() < 2U)) {
    RCLCPP_WARN_THROTTLE(logger_,
      *rclcpp::Clock::make_shared(),
      2000,
      "[MincoPlanner] Local seed path is outside ROGMap boundary or too short after clipping.");
    return seed;
  }
  if (seed.dense_path.size() < 2U) {
    return seed;
  }

  seed.local_end_is_goal = (global_goal - seed.dense_path.back()).head<2>().norm() <= traj_goal_tolerance_;
  seed.sparse_waypoints = utils::getSparseWaypoints(seed.dense_path,
    max_vel_,
    max_acc_,
    seed.local_end_is_goal,
    [&mode_context](const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
      return isLineFree(mode_context.sparsifyQuery(), a, b);
    });
  seed.valid = seed.sparse_waypoints.size() >= 2U;
  return seed;
}

std::vector<Eigen::Vector3d> LocalPathProcessor::extractLocalPath(
  const std::vector<geometry_msgs::msg::PoseStamped> & global_path, const Eigen::Vector3d & cur_pos) const
{
  std::vector<Eigen::Vector3d> local_segment;
  if (global_path.empty()) {
    return local_segment;
  }

  size_t start_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_path.size(); ++i) {
    minco_planner::PlanningDeadline::check();
    const auto & pt = global_path[i].pose.position;
    double dist_sq =
      (cur_pos.x() - pt.x) * (cur_pos.x() - pt.x) + (cur_pos.y() - pt.y) * (cur_pos.y() - pt.y);
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      start_idx = i;
    }
  }

  double accum_dist = 0.0;
  local_segment.push_back(
    Eigen::Vector3d(global_path[start_idx].pose.position.x, global_path[start_idx].pose.position.y, 0.0));

  for (size_t i = start_idx + 1; i < global_path.size(); ++i) {
    minco_planner::PlanningDeadline::check();
    const auto & p1 = global_path[i - 1].pose.position;
    const auto & p2 = global_path[i].pose.position;
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    accum_dist += dist;

    local_segment.push_back(Eigen::Vector3d(p2.x, p2.y, 0.0));

    if (accum_dist >= lookahead_dist_) {
      break;
    }
  }

  return local_segment;
}

bool LocalPathProcessor::clipLocalPathByRogBoundary(
  std::vector<Eigen::Vector3d> & path, const PlannerModeContext & mode_context) const
{
  if (path.empty()) {
    return false;
  }

  const bool enable_clip =
    mode_context.mode() == PlannerMode::EXPLORATION || mode_context.clipSeedByRogBoundary();
  if (!enable_clip) {
    return path.size() >= 2U;
  }

  const auto query = mode_context.dynamicQuery();
  if (!query) {
    return false;
  }

  const double margin = mode_context.mode() == PlannerMode::PRIORMAP
                          ? mode_context.rogBoundaryMargin()
                          : mode_context.explorationBoundaryMargin();
  const int margin_cells =
    std::max(0, static_cast<int>(std::ceil(margin / std::max(1e-6, query->resolution()))));
  const int max_x = static_cast<int>(query->sizeX());
  const int max_y = static_cast<int>(query->sizeY());
  if (max_x <= 0 || max_y <= 0) {
    return false;
  }

  const double sample_step = mode_context.mode() == PlannerMode::PRIORMAP
                               ? mode_context.rogBoundarySampleStep()
                               : mode_context.explorationBoundarySampleStep();
  const double step = std::max(query->resolution(), std::max(1e-3, sample_step));

  auto inside_boundary = [&query, margin_cells, max_x, max_y](const Eigen::Vector3d & p) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!query->worldToMap(p.x(), p.y(), mx, my)) {
      return false;
    }
    const int ix = static_cast<int>(mx);
    const int iy = static_cast<int>(my);
    return ix >= margin_cells && iy >= margin_cells && ix < max_x - margin_cells &&
           iy < max_y - margin_cells;
  };

  std::vector<Eigen::Vector3d> clipped;
  clipped.reserve(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    minco_planner::PlanningDeadline::check();
    if (!inside_boundary(path[i])) {
      if (i == 0U) {
        path.clear();
        RCLCPP_WARN_THROTTLE(logger_,
          *rclcpp::Clock::make_shared(),
          2000,
          "[MincoPlanner] Local seed path starts outside ROGMap boundary; reject local replan seed.");
        return false;
      }
      break;
    }
    if (i > 0U) {
      const Eigen::Vector3d delta = path[i] - path[i - 1U];
      const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() / step)));
      for (int s = 1; s <= samples; ++s) {
        minco_planner::PlanningDeadline::check();
        const double ratio = static_cast<double>(s) / static_cast<double>(samples);
        if (!inside_boundary(path[i - 1U] + ratio * delta)) {
          path.swap(clipped);
          return path.size() >= 2U;
        }
      }
    }
    clipped.push_back(path[i]);
  }

  path.swap(clipped);
  return path.size() >= 2U;
}

}  // namespace minco_planner
