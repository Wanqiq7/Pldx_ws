#pragma once

#include <chrono>
#include <functional>
#include <stdexcept>

namespace minco_planner {

// A worker-scoped deadline shared by nested search/optimization loops. No ROS
// clock: cancellation must still work while simulation time is paused.
class PlanningInterrupted : public std::runtime_error
{
public:
  PlanningInterrupted() : std::runtime_error("planning cancelled or budget exhausted") {}
};

class PlanningDeadline
{
public:
  using Clock = std::chrono::steady_clock;
  PlanningDeadline(double seconds, std::function<bool()> cancelled)
  : PlanningDeadline(
      Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds)),
      std::move(cancelled))
  {
  }
  PlanningDeadline(Clock::time_point deadline, std::function<bool()> cancelled)
  : deadline_(deadline), cancelled_(std::move(cancelled)), previous_(current_)
  {
    current_ = this;
  }
  ~PlanningDeadline() { current_ = previous_; }
  PlanningDeadline(const PlanningDeadline &) = delete;
  PlanningDeadline & operator=(const PlanningDeadline &) = delete;

  static void check()
  {
    if (current_ && (Clock::now() >= current_->deadline_ || current_->cancelled_())) {
      throw PlanningInterrupted();
    }
  }

private:
  Clock::time_point deadline_;
  std::function<bool()> cancelled_;
  PlanningDeadline * previous_;
  inline static thread_local PlanningDeadline * current_{nullptr};
};

}  // namespace minco_planner
