#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lbr_demos_cpp {

constexpr std::size_t kJoints = 7;
using JointArray = std::array<double, kJoints>;
using Jacobian = Eigen::Matrix<double, 6, 7>;
using Vector6 = Eigen::Matrix<double, 6, 1>;

struct CartesianWallConfig {
  JointArray lower{};
  JointArray upper{};
  JointArray wall_stiffness{};
  JointArray wall_damping{};
  JointArray wall_torque_cap{};
  JointArray overlay_torque_cap{};
  JointArray overlay_torque_rate{};
  double activation{0.0};
  double full_wall_reserve{0.0};
  double startup_clearance{0.0};
  double prediction{0.0};
  double outward_task_scale{0.0};
  Vector6 cartesian_stiffness{Vector6::Zero()};
  Vector6 cartesian_damping{Vector6::Zero()};

  void validate() const {
    if (!std::isfinite(activation) || !std::isfinite(full_wall_reserve) ||
        !std::isfinite(startup_clearance) || !std::isfinite(prediction) ||
        !std::isfinite(outward_task_scale) ||
        activation <= full_wall_reserve || full_wall_reserve <= 0.0 || prediction < 0.0 ||
        startup_clearance < 0.0 || outward_task_scale < 0.0 || outward_task_scale > 1.0) {
      throw std::invalid_argument("Invalid Cartesian-wall scalar configuration");
    }
    if (!cartesian_stiffness.allFinite() || !cartesian_damping.allFinite() ||
        (cartesian_stiffness.array() < 0.0).any() ||
        (cartesian_damping.array() < 0.0).any()) {
      throw std::invalid_argument("Cartesian gains must be finite and non-negative");
    }
    for (std::size_t i = 0; i < kJoints; ++i) {
      if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) ||
          !std::isfinite(wall_stiffness[i]) || !std::isfinite(wall_damping[i]) ||
          !std::isfinite(wall_torque_cap[i]) || !std::isfinite(overlay_torque_cap[i]) ||
          !std::isfinite(overlay_torque_rate[i]) ||
          upper[i] - lower[i] <= 2.0 * (activation + startup_clearance) ||
          wall_stiffness[i] <= 0.0 || wall_damping[i] < 0.0 || wall_torque_cap[i] <= 0.0 ||
          overlay_torque_cap[i] <= 0.0 || overlay_torque_rate[i] <= 0.0) {
        throw std::invalid_argument("Invalid joint configuration at index " + std::to_string(i));
      }
    }
  }
};

struct CartesianWallInput {
  JointArray q{};
  JointArray dq{};
  Jacobian jacobian{Jacobian::Zero()};
  Eigen::Isometry3d current_pose{Eigen::Isometry3d::Identity()};
  double dt{0.0};
};

struct CartesianWallOutput {
  JointArray cartesian_torque{};
  JointArray wall_torque{};
  JointArray commanded_torque{};
  std::array<bool, kJoints> wall_active{};
};

class CartesianWallController {
 public:
  explicit CartesianWallController(CartesianWallConfig config) : config_(std::move(config)) {
    config_.validate();
  }

  void activate(const Eigen::Isometry3d &measured_pose) {
    if (!measured_pose.matrix().allFinite()) {
      throw std::invalid_argument("Cannot activate from a non-finite pose");
    }
    reference_pose_ = measured_pose;
    previous_command_.fill(0.0);
    active_ = true;
  }

  void deactivate() {
    active_ = false;
    previous_command_.fill(0.0);
  }

  bool active() const { return active_; }

  void validate_startup(const JointArray &q) const {
    for (std::size_t i = 0; i < kJoints; ++i) {
      const double margin = config_.activation + config_.startup_clearance;
      if (!std::isfinite(q[i]) || q[i] <= config_.lower[i] + margin ||
          q[i] >= config_.upper[i] - margin) {
        throw std::invalid_argument("Joint starts inside wall region at index " +
                                    std::to_string(i));
      }
    }
  }

  CartesianWallOutput update(const CartesianWallInput &input) {
    if (!active_) {
      throw std::logic_error("Controller update requested while inactive");
    }
    if (!std::isfinite(input.dt) || input.dt <= 0.0 || input.dt > 0.1 ||
        !input.jacobian.allFinite() || !input.current_pose.matrix().allFinite()) {
      throw std::invalid_argument("Invalid controller input or interval");
    }
    for (std::size_t i = 0; i < kJoints; ++i) {
      if (!std::isfinite(input.q[i]) || !std::isfinite(input.dq[i]) ||
          input.q[i] <= config_.lower[i] || input.q[i] >= config_.upper[i]) {
        throw std::invalid_argument("Invalid or out-of-range joint state");
      }
    }

    Vector6 error = Vector6::Zero();
    error.head<3>() = reference_pose_.translation() - input.current_pose.translation();
    Eigen::Quaterniond current(input.current_pose.linear());
    Eigen::Quaterniond reference(reference_pose_.linear());
    if (current.dot(reference) < 0.0) {
      current.coeffs() *= -1.0;
    }
    Eigen::Quaterniond orientation_error = current.conjugate() * reference;
    error.tail<3>() = input.current_pose.linear() *
                      Eigen::Vector3d(orientation_error.x(), orientation_error.y(),
                                      orientation_error.z()) * 2.0;

    Eigen::Matrix<double, 7, 1> dq;
    for (std::size_t i = 0; i < kJoints; ++i) dq[i] = input.dq[i];
    const Vector6 twist = input.jacobian * dq;
    const Vector6 wrench = config_.cartesian_stiffness.cwiseProduct(error) -
                           config_.cartesian_damping.cwiseProduct(twist);
    Eigen::Matrix<double, 7, 1> task = input.jacobian.transpose() * wrench;

    CartesianWallOutput output;
    for (std::size_t i = 0; i < kJoints; ++i) {
      double wall = 0.0;
      for (const int side : {-1, 1}) {
        const double distance = side > 0 ? config_.upper[i] - input.q[i]
                                         : input.q[i] - config_.lower[i];
        const double outward_speed = std::max(0.0, side * input.dq[i]);
        const double penetration =
            std::max(0.0, config_.activation - distance + config_.prediction * outward_speed);
        const double blend_position = std::min(penetration, config_.activation);
        const double x = std::clamp(blend_position /
                                        (config_.activation - config_.full_wall_reserve),
                                    0.0, 1.0);
        const double blend = x * x * (3.0 - 2.0 * x);
        wall -= side * blend *
                (config_.wall_stiffness[i] * penetration +
                 config_.wall_damping[i] * outward_speed);
      }
      wall = std::clamp(wall, -config_.wall_torque_cap[i], config_.wall_torque_cap[i]);
      output.wall_torque[i] = wall;
      output.wall_active[i] = std::abs(wall) > 0.0;

      double task_torque = task[static_cast<Eigen::Index>(i)];
      if (output.wall_active[i] && task_torque * wall < 0.0) {
        task_torque *= config_.outward_task_scale;
      }
      output.cartesian_torque[i] = task_torque;
      const double raw = std::clamp(task_torque + wall, -config_.overlay_torque_cap[i],
                                    config_.overlay_torque_cap[i]);
      const double delta = config_.overlay_torque_rate[i] * input.dt;
      double commanded =
          std::clamp(raw, previous_command_[i] - delta, previous_command_[i] + delta);
      // A normal slew limit cannot both preserve its rate bound and immediately remove a
      // previously commanded torque that points farther into a newly active wall.  The wall
      // wins that conflict: cut the outward residual to zero, then slew inward normally.
      if (output.wall_active[i] && commanded * wall < 0.0) commanded = 0.0;
      if (!std::isfinite(task_torque) || !std::isfinite(wall) || !std::isfinite(commanded)) {
        throw std::runtime_error("Controller produced non-finite torque");
      }
      output.commanded_torque[i] = commanded;
    }
    previous_command_ = output.commanded_torque;
    return output;
  }

 private:
  CartesianWallConfig config_;
  Eigen::Isometry3d reference_pose_{Eigen::Isometry3d::Identity()};
  JointArray previous_command_{};
  bool active_{false};
};

}  // namespace lbr_demos_cpp
