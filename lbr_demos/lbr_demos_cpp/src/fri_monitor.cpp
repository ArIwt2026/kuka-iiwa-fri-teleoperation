#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "friClientApplication.h"
#include "friException.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
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

  ~FRIMonitor() override {
    std::lock_guard<std::mutex> lock(csv_mutex_);
    if (csv_file_.is_open()) {
      csv_file_.flush();
      csv_file_.close();
    }
  }

  void handleRecordTrigger(const std::string &data, rclcpp::Logger logger) {
    std::lock_guard<std::mutex> lock(csv_mutex_);
    if (data.empty() || data == "STOP") {
      if (is_csv_recording_.load()) {
        is_csv_recording_.store(false);
        if (csv_file_.is_open()) {
          csv_file_.flush();
          csv_file_.close();
        }
        current_csv_path_.clear();
        RCLCPP_INFO(logger, "Direct KUKA FRI CSV recording stopped. Total samples: %lu", csv_sample_count_);
      }
    } else {
      std::string path = data;
      if (path.find(".csv") == std::string::npos) {
        if (!path.empty() && path.back() != '/') path += "/";
        path += "kuka_telemetry.csv";
      }
      if (is_csv_recording_.load() && csv_file_.is_open() && current_csv_path_ == path) {
        return; // Already recording to this file!
      }
      if (csv_file_.is_open()) {
        csv_file_.flush();
        csv_file_.close();
      }
      csv_file_.open(path, std::ios::out | std::ios::trunc);
      if (csv_file_.is_open()) {
        current_csv_path_ = path;
        writeCsvHeader();
        csv_start_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        csv_sample_count_ = 0;
        is_csv_recording_.store(true);
        RCLCPP_INFO(logger, "Direct KUKA FRI CSV recording started: %s", path.c_str());
      } else {
        RCLCPP_ERROR(logger, "Failed to open KUKA CSV file: %s", path.c_str());
      }
    }
  }

  void writeCsvHeader() {
    csv_file_ << "pc_epoch_ns,pc_steady_ns,rel_time_s,fri_sec,fri_nanosec,sample_time_s,"
              << "session_state,conn_quality,tracking_perf,output1";
    for (int i = 1; i <= 7; ++i) csv_file_ << ",q_meas_" << i;
    for (int i = 1; i <= 7; ++i) csv_file_ << ",q_cmd_" << i;
    for (int i = 1; i <= 7; ++i) csv_file_ << ",dq_est_" << i;
    for (int i = 1; i <= 7; ++i) csv_file_ << ",tau_meas_" << i;
    for (int i = 1; i <= 7; ++i) csv_file_ << ",tau_cmd_" << i;
    for (int i = 1; i <= 7; ++i) csv_file_ << ",tau_ext_" << i;
    csv_file_ << "\n";
  }

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

    const double dt = state.getSampleTime();
    std::vector<double> velocity(7, 0.0);
    if (velocity_init_ && dt > 0.0 && position) {
      for (int i = 0; i < 7; ++i) {
        velocity[i] = (position[i] - prev_position_[i]) / dt;
      }
    }
    if (position) {
      std::copy(position, position + 7, prev_position_.begin());
      velocity_init_ = true;
    }

    const auto stamp = rclcpp::Clock().now();
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp;
    for (int i = 1; i <= 7; ++i) {
      message.name.push_back(robot_name_ + "_A" + std::to_string(i));
    }
    message.position.assign(position, position + 7);
    message.velocity = velocity;
    message.effort.assign(effort, effort + 7);
    publisher_->publish(message);

    // Commanded Joint State
    const double *q_cmd = nullptr;
    const double *tau_cmd = nullptr;
    const double *tau_ext = nullptr;
    try { q_cmd = state.getCommandedJointPosition(); } catch (...) {}
    try { tau_cmd = state.getCommandedTorque(); } catch (...) {}
    try { tau_ext = state.getExternalTorque(); } catch (...) {}

    sensor_msgs::msg::JointState cmd_message;
    cmd_message.header.stamp = stamp;
    cmd_message.name = message.name;
    if (q_cmd) {
      cmd_message.position.assign(q_cmd, q_cmd + 7);
    } else {
      cmd_message.position.assign(position, position + 7);
    }
    if (tau_cmd) {
      cmd_message.effort.assign(tau_cmd, tau_cmd + 7);
    } else {
      cmd_message.effort.assign(7, 0.0);
    }
    command_publisher_->publish(cmd_message);

    // Direct 1 kHz CSV recording
    if (is_csv_recording_.load()) {
      std::lock_guard<std::mutex> lock(csv_mutex_);
      if (csv_file_.is_open()) {
        const uint64_t epoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const uint64_t steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const double rel_time_s = static_cast<double>(steady_ns - csv_start_ns_) * 1e-9;

        bool output1 = false;
        try { output1 = state.getBooleanIOValue("MediaFlange.Output1"); } catch (...) {}

        csv_file_ << epoch_ns << ","
                  << steady_ns << ","
                  << std::fixed << std::setprecision(6) << rel_time_s << ","
                  << state.getTimestampSec() << ","
                  << state.getTimestampNanoSec() << ","
                  << std::setprecision(5) << dt << ","
                  << static_cast<int>(state.getSessionState()) << ","
                  << static_cast<int>(state.getConnectionQuality()) << ","
                  << state.getTrackingPerformance() << ","
                  << (output1 ? 1 : 0);

        for (int i = 0; i < 7; ++i) csv_file_ << "," << std::setprecision(6) << (position ? position[i] : 0.0);
        for (int i = 0; i < 7; ++i) csv_file_ << "," << std::setprecision(6) << (q_cmd ? q_cmd[i] : (position ? position[i] : 0.0));
        for (int i = 0; i < 7; ++i) csv_file_ << "," << std::setprecision(6) << velocity[i];
        for (int i = 0; i < 7; ++i) csv_file_ << "," << std::setprecision(4) << (effort ? effort[i] : 0.0);
        for (int i = 0; i < 7; ++i) csv_file_ << "," << std::setprecision(4) << (tau_cmd ? tau_cmd[i] : 0.0);
        for (int i = 0; i < 7; ++i) csv_file_ << "," << std::setprecision(4) << (tau_ext ? tau_ext[i] : 0.0);
        csv_file_ << "\n";

        if (++csv_sample_count_ % 1000 == 0) {
          csv_file_.flush();
        }
      }
    }
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

  void setCommandPublisher(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr &pub) {
    command_publisher_ = pub;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
  std::string robot_name_;
  std::shared_ptr<std::atomic<int>> output_command_;
  std::shared_ptr<std::atomic_bool> current_readback_;
  std::optional<bool> last_readback_;
  std::mutex csv_mutex_;
  std::atomic<bool> is_csv_recording_{false};
  uint64_t csv_start_ns_{0};
  uint64_t csv_sample_count_{0};
  std::ofstream csv_file_;
  std::string current_csv_path_;
  std::array<double, 7> prev_position_{};
  bool velocity_init_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fri_monitor");
  const auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  const auto command_publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);

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
  client.setCommandPublisher(command_publisher);

  const auto record_trigger_sub = node->create_subscription<std_msgs::msg::String>(
      "/recorder/trigger", 10,
      [&client, node](const std_msgs::msg::String::SharedPtr msg) {
        client.handleRecordTrigger(msg->data, node->get_logger());
      });
  (void)record_trigger_sub;

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
