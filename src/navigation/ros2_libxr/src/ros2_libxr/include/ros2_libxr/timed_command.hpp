#pragma once

#include <chrono>
#include <optional>

namespace rm_serial_driver {

// Transport heartbeats never refresh the age of an upstream ROS command.
template <typename T> class TimedCommand
{
public:
  using Clock = std::chrono::steady_clock;

  void update(const T & value, Clock::time_point received_at = Clock::now())
  {
    value_ = value;
    received_at_ = received_at;
  }

  T value(Clock::time_point now) const
  {
    return received_at_ && now - *received_at_ < std::chrono::milliseconds(150) ? value_ : T{};
  }

private:
  T value_{};
  std::optional<Clock::time_point> received_at_;
};

}  // namespace rm_serial_driver
