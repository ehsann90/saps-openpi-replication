#include <franka/rate_limiting.h>
#include <joint_limits/joint_limits_helpers.hpp>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
  const auto require = [](bool ok) { if (!ok) throw std::runtime_error("offline limit probe failed"); };
  std::array<double, 7> previous{}, command{};
  command.fill(10000.0);
  auto up = franka::limitRate(franka::kMaxTorqueRate, command, previous);
  command.fill(-10000.0);
  auto down = franka::limitRate(franka::kMaxTorqueRate, command, previous);
  for (size_t i = 0; i < 7; ++i) {
    require(std::abs(up[i] - franka::kMaxTorqueRate[i] * franka::kDeltaT) < 1e-12);
    require(std::abs(down[i] + franka::kMaxTorqueRate[i] * franka::kDeltaT) < 1e-12);
  }
  command.fill(0.25);
  require(franka::limitRate(franka::kMaxTorqueRate, command, previous) == command);
  command[0] = std::numeric_limits<double>::quiet_NaN();
  bool rejected = false;
  try { franka::limitRate(franka::kMaxTorqueRate, command, previous); }
  catch (const std::invalid_argument&) { rejected = true; }
  require(rejected);
  // Synthetic previous torque, not an admissible robot operating point.
  previous.fill(100.0); command.fill(1000.0);
  auto not_absolute_clamped = franka::limitRate(franka::kMaxTorqueRate, command, previous);
  require(not_absolute_clamped[0] > 87.0);
  joint_limits::JointLimits no_effort_limit;
  auto unlimited = joint_limits::compute_effort_limits(no_effort_limit, 0.0, 0.0, 0.001);
  require(std::isinf(unlimited.lower_limit) && std::isinf(unlimited.upper_limit));
  joint_limits::JointLimits limits;
  limits.has_effort_limits = true; limits.max_effort = 87.0;
  limits.has_position_limits = true; limits.min_position = -2.7437; limits.max_position = 2.7437;
  limits.has_velocity_limits = true; limits.max_velocity = 2.62;
  auto bounded = joint_limits::compute_effort_limits(limits, 0.0, 0.0, 0.001);
  require(bounded.lower_limit == -87.0 && bounded.upper_limit == 87.0);
  auto at_speed = joint_limits::compute_effort_limits(limits, 0.0, 3.0, 0.001);
  std::cout << "{\"torque_rate_nm_s\":" << franka::kMaxTorqueRate[0]
            << ",\"nominal_period_s\":" << franka::kDeltaT
            << ",\"maximum_torque_step_nm\":" << up[0]
            << ",\"slew_is_not_absolute_clamp\":true,\"nonfinite_rejected\":true"
            << ",\"optional_effort_bounds\":[" << bounded.lower_limit << "," << bounded.upper_limit
            << "],\"optional_effort_bounds_above_speed\":[" << at_speed.lower_limit << "," << at_speed.upper_limit
            << "],\"robot_objects_created\":0,\"passed\":true}\n";
}
