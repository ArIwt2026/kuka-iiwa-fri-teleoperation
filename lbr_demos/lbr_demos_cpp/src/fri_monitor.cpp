#include <array>
#include <memory>
#include <string>

#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class FRIMonitor final : public KUKA::FRI::LBRClient {
public:
  FRIMonitor(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr &publisher,
             const std::string &robot_name)
      : publisher_(publisher), robot_name_(robot_name) {}

  void monitor() override {
    const auto &state = robotState();
    const double *position = state.getMeasuredJointPosition();
    const double *effort = state.getMeasuredTorque();

    sensor_msgs::msg::JointState message;
    message.header.stamp = rclcpp::Clock().now();
    for (int i = 1; i <= 7; ++i) {
      message.name.push_back(robot_name_ + "_A" + std::to_string(i));
    }
    message.position.assign(position, position + 7);
    message.effort.assign(effort, effort + 7);
    publisher_->publish(message);
  }

  void onStateChange(KUKA::FRI::ESessionState old_state,
                     KUKA::FRI::ESessionState new_state) override {
    RCLCPP_INFO(rclcpp::get_logger("fri_monitor"), "FRI state changed from %d to %d",
                static_cast<int>(old_state), static_cast<int>(new_state));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::string robot_name_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fri_monitor");
  const auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  const auto controller_ip = node->declare_parameter<std::string>("controller_ip", "172.31.1.147");
  const auto port = node->declare_parameter<int>("port", 30200);
  const auto robot_name = node->declare_parameter<std::string>("robot_name", "iiwa7");

  KUKA::FRI::UdpConnection connection;
  FRIMonitor client(publisher, robot_name);
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
