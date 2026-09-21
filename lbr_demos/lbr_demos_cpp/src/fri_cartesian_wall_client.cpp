#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/frames.hpp>

#include "friClientApplication.h"
#include "friException.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"
#include "lbr_fri_ros2/kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "lbr_demos_cpp/cartesian_wall_controller.hpp"

namespace {
constexpr char kOutputName[] = "MediaFlange.Output1";

template <std::size_t N>
std::array<double, N> read_array(rclcpp::Node &node, const std::string &name) {
  const auto values = node.declare_parameter<std::vector<double>>(name, std::vector<double>{});
  if (values.size() != N) {
    throw std::invalid_argument(name + " requires " + std::to_string(N) + " values");
  }
  std::array<double, N> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

Eigen::Isometry3d to_eigen(const KDL::Frame &frame) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index column = 0; column < 3; ++column) {
      result.linear()(row, column) = frame.M(row, column);
    }
  }
  result.translation() = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());
  return result;
}
}  // namespace

class FriCartesianWallClient final : public KUKA::FRI::LBRClient {
 public:
  FriCartesianWallClient(rclcpp::Node &node, const std::string &robot_description)
      : node_(node),
        kinematics_(robot_description, node.declare_parameter("chain_root", "iiwa7_link_0"),
                    node.declare_parameter("chain_tip", "iiwa7_link_ee")),
        controller_(load_config(node)) {
    velocity_filter_ = node_.declare_parameter("velocity_filter", 0.2);
    feedback_timeout_ms_ = node_.declare_parameter("feedback_timeout_ms", 100);
    if (!std::isfinite(velocity_filter_) || velocity_filter_ <= 0.0 || velocity_filter_ > 1.0) {
      throw std::invalid_argument("velocity_filter must be in (0, 1]");
    }
    if (feedback_timeout_ms_ <= 0 || feedback_timeout_ms_ > 1000) {
      throw std::invalid_argument("feedback_timeout_ms must be in [1, 1000]");
    }
    joint_state_publisher_ =
        node_.create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10));
    output_toggle_service_ = node_.create_service<std_srvs::srv::Trigger>(
        "/fri/mediaflange_output1/toggle",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          if (!feedback_is_fresh() || !has_output_readback_.load(std::memory_order_acquire)) {
            response->success = false;
            response->message = "No fresh MediaFlange.Output1 readback; request rejected.";
            return;
          }
          const bool was_requested = run_requested_.load(std::memory_order_acquire);
          const int previous = output_command_.load(std::memory_order_relaxed);
          const bool readback = output_readback_.load(std::memory_order_relaxed);
          if (!was_requested && previous >= 0 && (previous == 1) != readback) {
            response->success = false;
            response->message = "Previous Output1 request is not confirmed; START rejected.";
            return;
          }
          const bool next = previous >= 0 ? previous == 0
                                          : !readback;
          const bool request_run = !was_requested;
          if (request_run) fault_latched_.store(false, std::memory_order_release);
          run_requested_.store(request_run, std::memory_order_release);
          if (!request_run) {
            commanding_authorized_.store(false, std::memory_order_release);
          }
          output_command_.store(next ? 1 : 0, std::memory_order_release);
          response->success = true;
          response->message = std::string(request_run ? "START" : "STOP") +
                              " requested via MediaFlange.Output1=" +
                              (next ? "true" : "false") + ".";
        });
  }

  void publish_joint_state() {
    if (!feedback_is_fresh()) {
      has_state_.store(false, std::memory_order_release);
      has_output_readback_.store(false, std::memory_order_release);
      return;
    }
    if (!has_state_.load(std::memory_order_acquire)) return;
    sensor_msgs::msg::JointState message;
    message.header.stamp = node_.now();
    for (std::size_t i = 0; i < lbr_demos_cpp::kJoints; ++i) {
      message.name.emplace_back(robot_name_ + "_A" + std::to_string(i + 1));
      message.position.push_back(position_[i].load(std::memory_order_relaxed));
      message.velocity.push_back(velocity_[i].load(std::memory_order_relaxed));
      message.effort.push_back(effort_[i].load(std::memory_order_relaxed));
    }
    joint_state_publisher_->publish(message);
  }

  void set_robot_name(std::string robot_name) { robot_name_ = std::move(robot_name); }

  void mark_disconnected() {
    commanding_authorized_.store(false, std::memory_order_release);
    run_requested_.store(false, std::memory_order_release);
    has_state_.store(false, std::memory_order_release);
    has_output_readback_.store(false, std::memory_order_release);
    controller_.deactivate();
    wrench_command_.fill(0.0);
  }

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override {
    controller_.deactivate();
    velocity_initialized_ = false;
    filtered_dq_.fill(0.0);
    wrench_command_.fill(0.0);
    commanding_authorized_.store(false, std::memory_order_release);
    const bool interrupted =
        (old_state == KUKA::FRI::COMMANDING_ACTIVE ||
         old_state == KUKA::FRI::COMMANDING_WAIT) &&
        new_state != KUKA::FRI::COMMANDING_ACTIVE;
    if (interrupted && run_requested_.exchange(false, std::memory_order_acq_rel)) {
      fault_latched_.store(true, std::memory_order_release);
      RCLCPP_ERROR(node_.get_logger(),
                   "FRI commanding ended before STOP; controller fault-latched until a new START");
    }
    if (new_state == KUKA::FRI::COMMANDING_ACTIVE) {
      const bool authorized = run_requested_.load(std::memory_order_acquire) &&
                              !fault_latched_.load(std::memory_order_acquire);
      commanding_authorized_.store(authorized, std::memory_order_release);
      if (!authorized) {
        fault_latched_.store(true, std::memory_order_release);
        RCLCPP_ERROR(node_.get_logger(),
                     "Unrequested FRI COMMANDING_ACTIVE; only zero overlay torque will be sent");
      }
    }
    RCLCPP_INFO(node_.get_logger(), "FRI state changed from %d to %d; controller reset",
                static_cast<int>(old_state), static_cast<int>(new_state));
  }

  void monitor() override {
    sample_state();
    apply_output_command();
  }

  void waitForCommand() override {
    const auto state = sample_state();
    robotCommand().setJointPosition(state.q.data());
    wrench_command_.fill(0.0);
    robotCommand().setWrench(wrench_command_.data());
    apply_output_command();
  }

  void command() override {
    if (robotState().getClientCommandMode() != KUKA::FRI::EClientCommandMode::WRENCH ||
        robotState().getControlMode() != KUKA::FRI::EControlMode::CART_IMP_CONTROL_MODE) {
      throw std::runtime_error("Expected FRI WRENCH with CART_IMP_CONTROL_MODE");
    }
    const auto state = sample_state();
    robotCommand().setJointPosition(state.q.data());
    if (!commanding_authorized_.load(std::memory_order_acquire) ||
        fault_latched_.load(std::memory_order_acquire)) {
      controller_.deactivate();
      wrench_command_.fill(0.0);
      robotCommand().setWrench(wrench_command_.data());
      apply_output_command();
      return;
    }
    const auto pose = to_eigen(kinematics_.compute_fk(state.q));
    if (!controller_.active()) {
      try {
        controller_.validate_startup(state.q);
        controller_.activate(pose);
      } catch (const std::exception &exception) {
        fault_latched_.store(true, std::memory_order_release);
        run_requested_.store(false, std::memory_order_release);
        commanding_authorized_.store(false, std::memory_order_release);
        wrench_command_.fill(0.0);
        const bool readback = output_readback_.load(std::memory_order_relaxed);
        output_command_.store(readback ? 0 : 1, std::memory_order_release);
        robotCommand().setWrench(wrench_command_.data());
        apply_output_command();
        RCLCPP_ERROR(node_.get_logger(), "Controller START refused: %s", exception.what());
        return;
      }
    }

    lbr_demos_cpp::CartesianWallInput input;
    input.q = state.q;
    input.dq = state.dq;
    input.current_pose = pose;
    input.dt = state.dt;
    const auto &kdl_jacobian = kinematics_.compute_jacobian(state.q);
    input.jacobian = kdl_jacobian.data;
    const auto output = controller_.update(input);
    for (int i = 0; i < 6; ++i) {
      wrench_command_[static_cast<std::size_t>(i)] = output.commanded_wrench[i];
    }
    robotCommand().setWrench(wrench_command_.data());
    apply_output_command();
  }

 private:
  struct Sample {
    lbr_demos_cpp::JointArray q{};
    lbr_demos_cpp::JointArray dq{};
    double dt{0.0};
  };

  static lbr_demos_cpp::CartesianWallConfig load_config(rclcpp::Node &node) {
    lbr_demos_cpp::CartesianWallConfig config;
    config.lower = read_array<7>(node, "limit_lower");
    config.upper = read_array<7>(node, "limit_upper");
    config.wall_stiffness = read_array<7>(node, "limit_stiffness");
    config.wall_damping = read_array<7>(node, "limit_damping");
    config.wall_torque_cap = read_array<7>(node, "limit_torque_cap");
    config.overlay_torque_cap = read_array<7>(node, "overlay_torque_cap");
    config.overlay_torque_rate = read_array<7>(node, "overlay_torque_rate");
    config.activation = node.declare_parameter("limit_activation_deg", 10.0) * M_PI / 180.0;
    config.full_wall_reserve = node.declare_parameter("limit_reserve_deg", 3.0) * M_PI / 180.0;
    config.startup_clearance =
        node.declare_parameter("startup_clearance_deg", 1.0) * M_PI / 180.0;
    config.prediction = node.declare_parameter("limit_prediction_seconds", 0.1);
    config.outward_task_scale = node.declare_parameter("outward_task_scale", 0.0);
    config.force_cap = node.declare_parameter("force_cap", 30.0);
    config.torque_cap = node.declare_parameter("torque_cap", 10.0);
    config.wrench_rate = node.declare_parameter("wrench_rate", 100.0);
    const auto stiffness = read_array<6>(node, "cartesian_stiffness");
    const auto damping = read_array<6>(node, "cartesian_damping");
    for (Eigen::Index i = 0; i < 6; ++i) {
      config.cartesian_stiffness[i] = stiffness[static_cast<std::size_t>(i)];
      config.cartesian_damping[i] = damping[static_cast<std::size_t>(i)];
    }
    return config;
  }

  Sample sample_state() {
    Sample sample;
    const double *q = robotState().getMeasuredJointPosition();
    const double *tau = robotState().getMeasuredTorque();
    const double nominal_dt = robotState().getSampleTime();
    if (!std::isfinite(nominal_dt) || nominal_dt <= 0.0 || nominal_dt > 0.1) {
      throw std::runtime_error("Invalid FRI sample time");
    }
    const auto now = std::chrono::steady_clock::now();
    const std::uint64_t timestamp_ns =
        static_cast<std::uint64_t>(robotState().getTimestampSec()) * 1000000000ULL +
        static_cast<std::uint64_t>(robotState().getTimestampNanoSec());
    double dt = nominal_dt;
    if (velocity_initialized_) {
      if (timestamp_ns > previous_robot_timestamp_ns_) {
        const double computed_dt =
            static_cast<double>(timestamp_ns - previous_robot_timestamp_ns_) * 1e-9;
        if (std::isfinite(computed_dt) &&
            computed_dt <= static_cast<double>(feedback_timeout_ms_) * 1e-3) {
          dt = computed_dt;
        } else {
          // Feedback gap exceeded timeout: reset velocity estimator and fault if commanding
          velocity_initialized_ = false;
          filtered_dq_.fill(0.0);
          if (commanding_authorized_.load(std::memory_order_acquire)) {
            commanding_authorized_.store(false, std::memory_order_release);
            fault_latched_.store(true, std::memory_order_release);
            controller_.deactivate();
          }
        }
      } else {
        // Timestamp jitter or duplicate: use nominal dt
        dt = nominal_dt;
      }
    }
    sample.dt = dt;
    for (std::size_t i = 0; i < lbr_demos_cpp::kJoints; ++i) {
      if (!std::isfinite(q[i]) || !std::isfinite(tau[i])) {
        throw std::runtime_error("Non-finite FRI state");
      }
      sample.q[i] = q[i];
      const double raw_velocity = velocity_initialized_ ? (q[i] - previous_q_[i]) / dt : 0.0;
      filtered_dq_[i] += velocity_filter_ * (raw_velocity - filtered_dq_[i]);
      sample.dq[i] = filtered_dq_[i];
      previous_q_[i] = q[i];
      position_[i].store(q[i], std::memory_order_relaxed);
      velocity_[i].store(sample.dq[i], std::memory_order_relaxed);
      effort_[i].store(tau[i], std::memory_order_relaxed);
    }
    previous_robot_timestamp_ns_ = timestamp_ns;
    velocity_initialized_ = true;
    last_feedback_ns_.store(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(),
        std::memory_order_release);
    has_state_.store(true, std::memory_order_release);
    try {
      const bool output = robotState().getBooleanIOValue(kOutputName);
      output_readback_.store(output, std::memory_order_relaxed);
      has_output_readback_.store(true, std::memory_order_release);
    } catch (const KUKA::FRI::FRIException &) {
      has_output_readback_.store(false, std::memory_order_release);
    }
    return sample;
  }

  bool feedback_is_fresh() const {
    const std::int64_t last = last_feedback_ns_.load(std::memory_order_acquire);
    if (last <= 0) return false;
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const std::int64_t current =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    const std::int64_t age = current - last;
    return age >= 0 && age <= static_cast<std::int64_t>(feedback_timeout_ms_) * 1000000LL;
  }

  void apply_output_command() {
    const int output = output_command_.load(std::memory_order_acquire);
    if (output >= 0) robotCommand().setBooleanIOValue(kOutputName, output == 1);
  }

  rclcpp::Node &node_;
  lbr_fri_ros2::Kinematics kinematics_;
  lbr_demos_cpp::CartesianWallController controller_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr output_toggle_service_;
  std::string robot_name_{"iiwa7"};
  double velocity_filter_{0.2};
  int feedback_timeout_ms_{100};
  lbr_demos_cpp::JointArray previous_q_{};
  lbr_demos_cpp::JointArray filtered_dq_{};
  std::array<double, 6> wrench_command_{};
  bool velocity_initialized_{false};
  std::uint64_t previous_robot_timestamp_ns_{0};
  std::array<std::atomic<double>, 7> position_{};
  std::array<std::atomic<double>, 7> velocity_{};
  std::array<std::atomic<double>, 7> effort_{};
  std::atomic_bool has_state_{false};
  std::atomic_bool output_readback_{false};
  std::atomic_bool has_output_readback_{false};
  std::atomic<int> output_command_{-1};
  std::atomic_bool run_requested_{false};
  std::atomic_bool commanding_authorized_{false};
  std::atomic_bool fault_latched_{false};
  std::atomic<std::int64_t> last_feedback_ns_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fri_cartesian_wall_client");
  try {
    const auto controller_ip = node->declare_parameter<std::string>("controller_ip", "192.170.10.2");
    const auto port = node->declare_parameter<int>("port", 30200);
    const auto receive_timeout_ms = node->declare_parameter<int>("receive_timeout_ms", 0);
    const auto robot_name = node->declare_parameter<std::string>("robot_name", "iiwa7");
    const auto robot_description = node->declare_parameter<std::string>("robot_description", "");
    if (robot_description.empty()) throw std::invalid_argument("robot_description is required");
    if (receive_timeout_ms < 0 || receive_timeout_ms > 1000) {
      throw std::invalid_argument("receive_timeout_ms must be in [0, 1000]");
    }

    FriCartesianWallClient client(*node, robot_description);
    client.set_robot_name(robot_name);
    KUKA::FRI::UdpConnection connection(static_cast<unsigned int>(receive_timeout_ms));
    KUKA::FRI::ClientApplication application(connection, client);
    if (!application.connect(port, controller_ip.c_str())) {
      throw std::runtime_error("Could not connect to KUKA controller");
    }
    auto publish_timer = node->create_wall_timer(
        std::chrono::milliseconds(10), [&client] { client.publish_joint_state(); });
    (void)publish_timer;
    // Keep ROS allocation, service handling and publication off the FRI packet thread.
    std::thread ros_thread([node] { rclcpp::spin(node); });
    RCLCPP_INFO(node->get_logger(),
                "FRI Cartesian-wall client waiting for KUKA controller at %s:%d",
                controller_ip.c_str(), static_cast<int>(port));
    while (rclcpp::ok()) {
      try {
        if (!application.step()) {
          client.mark_disconnected();
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
      } catch (const KUKA::FRI::FRIException &exception) {
        client.mark_disconnected();
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "FRI communication error: %s", exception.getErrorMessage());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      } catch (const std::exception &exception) {
        client.mark_disconnected();
        RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                             "FRI cycle exception: %s", exception.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    client.mark_disconnected();
    application.disconnect();
    rclcpp::shutdown();
    if (ros_thread.joinable()) {
      ros_thread.join();
    }
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(node->get_logger(), "FRI Cartesian-wall client stopped: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  } catch (const KUKA::FRI::FRIException &exception) {
    RCLCPP_FATAL(node->get_logger(), "FRI Cartesian-wall client stopped: %s",
                 exception.getErrorMessage());
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    RCLCPP_FATAL(node->get_logger(), "FRI Cartesian-wall client stopped: unknown exception");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
