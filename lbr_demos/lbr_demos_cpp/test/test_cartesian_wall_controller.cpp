#include <cmath>

#include <gtest/gtest.h>

#include "lbr_demos_cpp/cartesian_wall_controller.hpp"

namespace {
using lbr_demos_cpp::CartesianWallConfig;
using lbr_demos_cpp::CartesianWallController;
using lbr_demos_cpp::CartesianWallInput;

CartesianWallConfig config() {
  CartesianWallConfig value;
  value.lower.fill(-2.0);
  value.upper.fill(2.0);
  value.wall_stiffness.fill(20.0);
  value.wall_damping.fill(2.0);
  value.wall_torque_cap.fill(5.0);
  value.overlay_torque_cap.fill(6.0);
  value.overlay_torque_rate.fill(100.0);
  value.activation = 0.2;
  value.full_wall_reserve = 0.05;
  value.startup_clearance = 0.01;
  value.prediction = 0.1;
  value.outward_task_scale = 0.0;
  return value;
}

CartesianWallInput input() {
  CartesianWallInput value;
  value.q.fill(0.0);
  value.dq.fill(0.0);
  value.dt = 0.01;
  return value;
}
}  // namespace

TEST(CartesianWallController, ZeroGainsAndFreeJointsProduceZeroTorque) {
  CartesianWallController controller(config());
  controller.activate(Eigen::Isometry3d::Identity());
  const auto output = controller.update(input());
  for (std::size_t i = 0; i < lbr_demos_cpp::kJoints; ++i) {
    EXPECT_DOUBLE_EQ(output.commanded_torque[i], 0.0);
    EXPECT_FALSE(output.wall_active[i]);
  }
}

TEST(CartesianWallController, UpperWallPushesInwardAndDoesNotBrakeRetreat) {
  CartesianWallController controller(config());
  controller.activate(Eigen::Isometry3d::Identity());
  auto state = input();
  state.q[0] = 1.9;
  state.dq[0] = 0.5;
  const auto outward = controller.update(state);
  EXPECT_LT(outward.wall_torque[0], 0.0);

  controller.activate(Eigen::Isometry3d::Identity());
  state.dq[0] = -0.5;
  const auto retreat = controller.update(state);
  EXPECT_LT(retreat.wall_torque[0], 0.0);
  EXPECT_LT(std::abs(retreat.wall_torque[0]), std::abs(outward.wall_torque[0]));
}

TEST(CartesianWallController, LowerWallPushesInward) {
  CartesianWallController controller(config());
  controller.activate(Eigen::Isometry3d::Identity());
  auto state = input();
  state.q[2] = -1.9;
  state.dq[2] = -0.5;
  EXPECT_GT(controller.update(state).wall_torque[2], 0.0);
}

TEST(CartesianWallController, CartesianTranslationUsesJacobianTranspose) {
  auto settings = config();
  settings.cartesian_stiffness[0] = 100.0;
  settings.overlay_torque_rate.fill(10000.0);
  CartesianWallController controller(settings);
  controller.activate(Eigen::Isometry3d::Identity());
  auto state = input();
  state.current_pose.translation().x() = -0.01;
  state.jacobian(0, 3) = 1.0;
  const auto output = controller.update(state);
  EXPECT_NEAR(output.cartesian_torque[3], 1.0, 1e-12);
  EXPECT_NEAR(output.commanded_torque[3], 1.0, 1e-12);
}

TEST(CartesianWallController, WallSuppressesConflictingTaskTorque) {
  auto settings = config();
  settings.cartesian_stiffness[0] = 100.0;
  settings.overlay_torque_rate.fill(10000.0);
  CartesianWallController controller(settings);
  controller.activate(Eigen::Isometry3d::Identity());
  auto state = input();
  state.q[0] = 1.9;
  state.current_pose.translation().x() = -0.01;
  state.jacobian(0, 0) = 1.0;
  const auto output = controller.update(state);
  EXPECT_LT(output.wall_torque[0], 0.0);
  EXPECT_DOUBLE_EQ(output.cartesian_torque[0], 0.0);
  EXPECT_LT(output.commanded_torque[0], 0.0);
}

TEST(CartesianWallController, WallImmediatelyRemovesRateLimitedOutwardResidual) {
  auto settings = config();
  settings.cartesian_stiffness[0] = 100.0;
  settings.overlay_torque_rate.fill(50.0);
  CartesianWallController controller(settings);
  controller.activate(Eigen::Isometry3d::Identity());
  auto state = input();
  state.q[0] = 1.79;
  state.current_pose.translation().x() = -0.05;
  state.jacobian(0, 0) = 1.0;
  for (int sample = 0; sample < 10; ++sample) controller.update(state);

  state.q[0] = 1.801;
  state.dq[0] = 1.1;
  const auto output = controller.update(state);
  EXPECT_LT(output.wall_torque[0], 0.0);
  EXPECT_DOUBLE_EQ(output.cartesian_torque[0], 0.0);
  EXPECT_LE(output.commanded_torque[0], 0.0);
}

TEST(CartesianWallController, UsesPerJointWallStiffness) {
  auto settings = config();
  settings.wall_stiffness[0] = 80.0;
  settings.wall_stiffness[4] = 10.0;
  settings.wall_damping.fill(0.0);
  settings.overlay_torque_rate.fill(10000.0);
  CartesianWallController controller(settings);
  controller.activate(Eigen::Isometry3d::Identity());
  auto state = input();
  state.q[0] = 1.9;
  state.q[4] = 1.9;
  const auto output = controller.update(state);
  EXPECT_LT(output.wall_torque[0], output.wall_torque[4]);
  EXPECT_GT(std::abs(output.wall_torque[0]), std::abs(output.wall_torque[4]));
}

TEST(CartesianWallController, LimitsTorqueRate) {
  auto settings = config();
  settings.overlay_torque_rate.fill(2.0);
  CartesianWallController controller(settings);
  controller.activate(Eigen::Isometry3d::Identity());
  auto state = input();
  state.q[0] = 1.96;
  const auto output = controller.update(state);
  EXPECT_NEAR(output.commanded_torque[0], -0.02, 1e-12);
}

TEST(CartesianWallController, RejectsPhysicalLimitAndInvalidTime) {
  CartesianWallController controller(config());
  controller.activate(Eigen::Isometry3d::Identity());
  auto state = input();
  state.q[0] = 2.0;
  EXPECT_THROW(controller.update(state), std::invalid_argument);
  state = input();
  state.dt = 0.0;
  EXPECT_THROW(controller.update(state), std::invalid_argument);
}

TEST(CartesianWallController, RejectsStartupInsideEitherWall) {
  CartesianWallController controller(config());
  lbr_demos_cpp::JointArray q{};
  controller.validate_startup(q);
  q[0] = 1.8;
  EXPECT_THROW(controller.validate_startup(q), std::invalid_argument);
  q[0] = -1.8;
  EXPECT_THROW(controller.validate_startup(q), std::invalid_argument);
}
