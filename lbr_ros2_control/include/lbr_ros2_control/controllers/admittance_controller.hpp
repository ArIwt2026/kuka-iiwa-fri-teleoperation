#ifndef LBR_ROS2_CONTROL__ADMITTANCE_CONTROLLER_HPP_
#define LBR_ROS2_CONTROL__ADMITTANCE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semantic_components/force_torque_sensor.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "friLBRState.h"
#include "lbr_fri_ros2/control.hpp"
#include "lbr_fri_ros2/types.hpp"
#include "lbr_ros2_control/system_interface_type_values.hpp"

namespace lbr_ros2_control {
class AdmittanceController : public controller_interface::ControllerInterface {
public:
  enum class TeachMode : uint8_t { HOME_HOLD = 0, ADMITTANCE = 1, RETURN_HOME = 2 };

  AdmittanceController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

protected:
  bool reference_state_interfaces_();
  void clear_state_interfaces_();
  void configure_joint_names_();
  void configure_admittance_impl_();
  void configure_inv_jac_ctrl_impl_();
  void zero_all_values_();
  void log_info_() const;
  void toggle_teach_mode_(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // robot description
  std::string robot_description_;

  // admittance
  bool initialized_ = false;
  std::unique_ptr<lbr_fri_ros2::AdmittanceImpl> admittance_impl_ptr_;
  Eigen::Matrix<double, 3, 1> t_init_, t_, t_prev_; // translation
  Eigen::Quaterniond r_init_, r_, r_prev_;          // rotation
  Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> f_ext_, delta_x_, dx_, ddx_;

  // joint veloctiy computation
  std::unique_ptr<lbr_fri_ros2::InvJacCtrlImpl> inv_jac_ctrl_impl_ptr_;
  lbr_fri_ros2::jnt_array_t q_, dq_;
  Eigen::Matrix<double, lbr_fri_ros2::CARTESIAN_DOF, 1> twist_command_;

  // interfaces
  lbr_fri_ros2::jnt_name_array_t joint_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      joint_position_state_interfaces_;
  std::unique_ptr<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      sample_time_state_interface_ptr_;
  std::unique_ptr<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      session_state_interface_ptr_;
  std::unique_ptr<semantic_components::ForceTorqueSensor> estimated_ft_sensor_ptr_;

  std::array<double, lbr_fri_ros2::N_JNTS> home_joint_positions_{};
  std::array<double, lbr_fri_ros2::N_JNTS> return_start_positions_{};
  std::atomic<TeachMode> requested_teach_mode_{TeachMode::RETURN_HOME};
  TeachMode active_teach_mode_{TeachMode::RETURN_HOME};
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> teach_mode_service_;
  double return_elapsed_{0.0};
  double return_duration_{2.0};
  double return_max_joint_velocity_{0.15};
  bool session_active_prev_{false};
};
} // namespace lbr_ros2_control
#endif // LBR_ROS2_CONTROL__ADMITTANCE_CONTROLLER_HPP_
