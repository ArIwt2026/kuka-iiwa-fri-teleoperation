#include <array>
#include <atomic>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include "friClientApplication.h"
#include "friException.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <iostream>

class FRIMonitor final : public KUKA::FRI::LBRClient {
public:
  FRIMonitor(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr &publisher,
             const std::string &robot_name,
             const std::shared_ptr<std::atomic<int>> &output_command,
             const std::shared_ptr<std::atomic_bool> &current_readback)
      : publisher_(publisher), robot_name_(robot_name),
        output_command_(output_command), current_readback_(current_readback) {}

  void monitor() override {
    const auto &state = robotState();
    const double *position = state.getMeasuredJointPosition();
    const double *effort = state.getMeasuredTorque();

    try {
      // 1. Fetch current value from robot and store it
      const bool current_output = state.getBooleanIOValue("MediaFlange.Output1");
      current_readback_->store(current_output);

      if (!last_readback_.has_value() || last_readback_.value() != current_output) {
        std::cout << "MediaFlange.Output1: " << std::boolalpha << current_output << std::endl;
        last_readback_ = current_output;
      }

      // 2. At init, output_command is -1 -> DO NOTHING.
      // Only command if user explicitly called the toggle service.
      const int cmd = output_command_->load();
      if (cmd >= 0) {
        robotCommand().setBooleanIOValue(
            "MediaFlange.Output1", cmd == 1);
      }
    } catch (const KUKA::FRI::FRIException &exception) {
      std::cerr << "Could not read/command MediaFlange.Output1: "
                << exception.getErrorMessage() << std::endl;
    }

    sensor_msgs::msg::JointState message;
    message.header.stamp = rclcpp::Clock().now();
    for (int i = 1; i <= 7; ++i) {
      message.name.push_back(robot_name_ + "_A" + std::to_string(i));
    }
    message.position.assign(position, position + 7);
    message.effort.assign(effort, effort + 7);
    publisher_->publish(message);
  }

  void waitForCommand() override {
    robotCommand().setJointPosition(robotState().getMeasuredJointPosition());
    if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::TORQUE) {
      const std::array<double, 7> zero_torques{};
      robotCommand().setTorque(zero_torques.data());
    }
    monitor();
  }

  void command() override {
    robotCommand().setJointPosition(robotState().getMeasuredJointPosition());
    if (robotState().getClientCommandMode() == KUKA::FRI::EClientCommandMode::TORQUE) {
      const std::array<double, 7> zero_torques{};
      robotCommand().setTorque(zero_torques.data());
    }
    monitor();
  }

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override {
    RCLCPP_INFO(rclcpp::get_logger("fri_monitor"), "FRI state changed from %d to %d",
                static_cast<int>(old_state), static_cast<int>(new_state));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::string robot_name_;
  std::shared_ptr<std::atomic<int>> output_command_;
  std::shared_ptr<std::atomic_bool> current_readback_;
  std::optional<bool> last_readback_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fri_monitor");
  const auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  const auto controller_ip = node->declare_parameter<std::string>("controller_ip", "192.170.10.2");
  const auto port = node->declare_parameter<int>("port", 30200);
  const auto robot_name = node->declare_parameter<std::string>("robot_name", "iiwa7");

  const auto current_readback = std::make_shared<std::atomic_bool>(false);
  // -1 = uninitialized. At startup, only fetch current value and do nothing.
  const auto output_command = std::make_shared<std::atomic<int>>(-1);

  const auto fri_toggle_service = node->create_service<std_srvs::srv::Trigger>(
      "/fri/mediaflange_output1/toggle",
      [output_command, current_readback](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const int current_cmd = output_command->load();
        // If not commanded yet, toggle the opposite of the fetched robot state.
        // Otherwise, flip the last commanded value.
        const bool next_val = (current_cmd >= 0) ? !(current_cmd == 1) : !current_readback->load();
        output_command->store(next_val ? 1 : 0);
        response->success = true;
        response->message = std::string("MediaFlange.Output1 = ") +
                            (next_val ? "true" : "false");
      });

  (void)fri_toggle_service;

  KUKA::FRI::UdpConnection connection;
  FRIMonitor client(publisher, robot_name, output_command, current_readback);
  KUKA::FRI::ClientApplication application(connection, client);

  RCLCPP_INFO(node->get_logger(), "Listening for FRI monitor data from %s:%d",
              controller_ip.c_str(), static_cast<int>(port));
  if (!application.connect(port, controller_ip.c_str())) {
    RCLCPP_ERROR(node->get_logger(), "Could not connect to KUKA controller");
    rclcpp::shutdown();
    return 1;
  }

  while (rclcpp::ok() && application.step()) {
    rclcpp::spin_some(node);
  }

  application.disconnect();
  rclcpp::shutdown();
  return 0;
}
