#include "lbr_ros2_control/controllers/admittance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace lbr_ros2_control {
AdmittanceController::AdmittanceController() {}

controller_interface::InterfaceConfiguration
AdmittanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
AdmittanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // joint position interface
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  // estimated force-torque sensor interface
  for (const auto &interface_name : estimated_ft_sensor_ptr_->get_state_interface_names()) {
    interface_configuration.names.push_back(interface_name);
  }

  // additional state interfaces
  interface_configuration.names.push_back(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                          HW_IF_SAMPLE_TIME);
  interface_configuration.names.push_back(std::string(HW_IF_AUXILIARY_PREFIX) + "/" +
                                          HW_IF_SESSION_STATE);
  return interface_configuration;
}

controller_interface::CallbackReturn AdmittanceController::on_init() {
  try {
    if (!this->get_node()->has_parameter("robot_description")) {
      this->get_node()->declare_parameter("robot_description", "");
    }
    if (!this->get_node()->has_parameter("robot_name")) {
      this->get_node()->declare_parameter("robot_name", "lbr");
    }
    if (!this->get_node()->has_parameter("admittance.mass")) {
      this->get_node()->declare_parameter("admittance.mass",
                                          std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 1.0));
    }
    if (!this->get_node()->has_parameter("admittance.damping")) {
      this->get_node()->declare_parameter("admittance.damping",
                                          std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 0.0));
    }
    if (!this->get_node()->has_parameter("admittance.stiffness")) {
      this->get_node()->declare_parameter("admittance.stiffness",
                                          std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 0.0));
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.chain_root")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.chain_root", "lbr_link_0");
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.chain_tip")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.chain_tip", "lbr_link_ee");
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.damping")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.damping", 0.2);
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.max_linear_velocity")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.max_linear_velocity", 0.1);
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.max_angular_velocity")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.max_angular_velocity", 0.1);
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.joint_gains")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.joint_gains",
                                          std::vector<double>(lbr_fri_ros2::N_JNTS, 0.0));
    }
    if (!this->get_node()->has_parameter("inv_jac_ctrl.cartesian_gains")) {
      this->get_node()->declare_parameter("inv_jac_ctrl.cartesian_gains",
                                          std::vector<double>(lbr_fri_ros2::CARTESIAN_DOF, 0.0));
    }
    if (!this->get_node()->has_parameter("home_joint_positions_rad")) {
      this->get_node()->declare_parameter("home_joint_positions_rad", std::vector<double>{});
    }
    if (!this->get_node()->has_parameter("return_max_joint_velocity_rad_s")) {
      this->get_node()->declare_parameter("return_max_joint_velocity_rad_s", 0.15);
    }
    robot_description_ = this->get_node()->get_parameter("robot_description").as_string();
    if (robot_description_.empty()) {
      throw std::runtime_error("No robot description provided");
    }
    configure_joint_names_();
    const auto home = this->get_node()->get_parameter("home_joint_positions_rad").as_double_array();
    if (home.size() != lbr_fri_ros2::N_JNTS) {
      throw std::runtime_error("home_joint_positions_rad must contain seven joint angles");
    }
    for (std::size_t i = 0; i < home.size(); ++i) {
      if (!std::isfinite(home[i])) {
        throw std::runtime_error("home_joint_positions_rad contains a non-finite value");
      }
      home_joint_positions_[i] = home[i];
    }
    return_max_joint_velocity_ =
        this->get_node()->get_parameter("return_max_joint_velocity_rad_s").as_double();
    if (!std::isfinite(return_max_joint_velocity_) || return_max_joint_velocity_ <= 0.0) {
      throw std::runtime_error("return_max_joint_velocity_rad_s must be positive and finite");
    }
    teach_mode_service_ = this->get_node()->create_service<std_srvs::srv::Trigger>(
        "~/teach_mode/toggle",
        std::bind(&AdmittanceController::toggle_teach_mode_, this,
                  std::placeholders::_1, std::placeholders::_2));
    configure_admittance_impl_();
    configure_inv_jac_ctrl_impl_();
    log_info_();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to initialize admittance controller with: %s.", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AdmittanceController::update(const rclcpp::Time & /*time*/,
                                                               const rclcpp::Duration &period) {
  // get estimated force-torque sensor values
  f_ext_.head(3) =
      Eigen::Map<Eigen::Matrix<double, 3, 1>>(estimated_ft_sensor_ptr_->get_forces().data());
  f_ext_.tail(3) =
      Eigen::Map<Eigen::Matrix<double, 3, 1>>(estimated_ft_sensor_ptr_->get_torques().data());

  // get joint positions
  std::for_each(q_.begin(), q_.end(), [&, i = 0](double &q_i) mutable {
    q_i = this->state_interfaces_[i].get_value();
    ++i;
  });

  const bool commanding_active =
      (static_cast<int>(session_state_interface_ptr_->get().get_value()) ==
       KUKA::FRI::ESessionState::COMMANDING_ACTIVE);
  if (!commanding_active) {
    session_active_prev_ = false;
    return controller_interface::return_type::OK;
  }

  if (!session_active_prev_) {
    session_active_prev_ = true;
    double max_delta = 0.0;
    for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
      return_start_positions_[i] = q_[i];
      max_delta = std::max(max_delta, std::abs(home_joint_positions_[i] - q_[i]));
    }
    if (max_delta < 1e-3) {
      active_teach_mode_ = TeachMode::HOME_HOLD;
      requested_teach_mode_.store(TeachMode::HOME_HOLD, std::memory_order_release);
    } else {
      // A quintic profile has a peak normalized velocity of 1.875.
      return_duration_ = std::max(2.0, 1.875 * max_delta / return_max_joint_velocity_);
      return_elapsed_ = 0.0;
      active_teach_mode_ = TeachMode::RETURN_HOME;
      requested_teach_mode_.store(TeachMode::RETURN_HOME, std::memory_order_release);
    }
  }

  // compute forward kinematics
  auto chain_tip_frame = inv_jac_ctrl_impl_ptr_->get_kinematics_ptr()->compute_fk(q_);
  t_ = Eigen::Map<Eigen::Matrix<double, 3, 1>>(chain_tip_frame.p.data);
  r_ = Eigen::Quaterniond(chain_tip_frame.M.data);

  const TeachMode requested_mode = requested_teach_mode_.load(std::memory_order_acquire);
  if (requested_mode != active_teach_mode_) {
    if (requested_mode == TeachMode::RETURN_HOME) {
      double max_delta = 0.0;
      for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
        return_start_positions_[i] = q_[i];
        max_delta = std::max(max_delta, std::abs(home_joint_positions_[i] - q_[i]));
      }
      // A quintic profile has a peak normalized velocity of 1.875.
      return_duration_ = std::max(2.0, 1.875 * max_delta / return_max_joint_velocity_);
      return_elapsed_ = 0.0;
    }
    if (requested_mode == TeachMode::ADMITTANCE) {
      initialized_ = false;
      zero_all_values_();
    }
    active_teach_mode_ = requested_mode;
  }

  if (active_teach_mode_ == TeachMode::HOME_HOLD) {
    for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
      command_interfaces_[i].set_value(home_joint_positions_[i]);
    }
    return controller_interface::return_type::OK;
  }

  if (active_teach_mode_ == TeachMode::RETURN_HOME) {
    return_elapsed_ = std::min(return_duration_, return_elapsed_ + period.seconds());
    const double u = return_elapsed_ / return_duration_;
    const double blend = 10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u;
    for (std::size_t i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
      command_interfaces_[i].set_value(
          return_start_positions_[i] + blend * (home_joint_positions_[i] - return_start_positions_[i]));
    }
    if (u >= 1.0) {
      active_teach_mode_ = TeachMode::HOME_HOLD;
      requested_teach_mode_.store(TeachMode::HOME_HOLD, std::memory_order_release);
    }
    return controller_interface::return_type::OK;
  }

  // compute steady state position and orientation. It is reset when entering
  // admittance so every teaching segment starts from the current robot pose.
  if (!initialized_) {
    t_init_ = t_;
    t_prev_ = t_init_;
    r_init_ = r_;
    r_prev_ = r_init_;
    initialized_ = true;
  }

  // compute translational delta and velocity
  delta_x_.head(3) = (t_ - t_init_);
  dx_.head(3) = (t_ - t_prev_) / period.seconds();

  // compute rotational delta and veloctity
  Eigen::AngleAxisd deltaa(r_.inverse() * r_init_);
  delta_x_.tail(3) = deltaa.axis() * deltaa.angle();
  Eigen::AngleAxisd da(r_.inverse() * r_prev_);
  dx_.tail(3) = da.axis() * da.angle();

  // update previous values
  t_prev_ = t_;
  r_prev_ = r_;

  // convert f_ext_ back to root frame
  f_ext_.head(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data).transpose() * f_ext_.head(3);
  f_ext_.tail(3) = Eigen::Matrix3d::Map(chain_tip_frame.M.data).transpose() * f_ext_.tail(3);

  // compute admittance
  admittance_impl_ptr_->compute(f_ext_, delta_x_, dx_, ddx_);

  // integrate ddx_ to command velocity
  twist_command_ = ddx_ * period.seconds();

  if (!inv_jac_ctrl_impl_ptr_) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Inverse Jacobian controller not initialized.");
    return controller_interface::return_type::ERROR;
  }
  // compute the joint velocity from the twist command target
  inv_jac_ctrl_impl_ptr_->compute(twist_command_, q_, dq_);

  // pass joint positions to hardware
  std::for_each(q_.begin(), q_.end(), [&, i = 0](const double &q_i) mutable {
    this->command_interfaces_[i].set_value(
        q_i + dq_[i] * sample_time_state_interface_ptr_->get().get_value());
    ++i;
  });

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
AdmittanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  estimated_ft_sensor_ptr_ = std::make_unique<semantic_components::ForceTorqueSensor>(
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_FORCE_X,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_FORCE_Y,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_FORCE_Z,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_TORQUE_X,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_TORQUE_Y,
      std::string(HW_IF_ESTIMATED_FT_PREFIX) + "/" + HW_IF_TORQUE_Z);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdmittanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!reference_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  zero_all_values_();
  initialized_ = false;
  session_active_prev_ = false;
  active_teach_mode_ = TeachMode::RETURN_HOME;
  requested_teach_mode_.store(TeachMode::RETURN_HOME, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
AdmittanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  clear_state_interfaces_();
  initialized_ = false;
  session_active_prev_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

void AdmittanceController::toggle_teach_mode_(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  TeachMode current = requested_teach_mode_.load(std::memory_order_acquire);
  while (true) {
    if (current == TeachMode::RETURN_HOME) {
      response->success = false;
      response->message = "Return to home is already in progress.";
      return;
    }
    const TeachMode next = current == TeachMode::HOME_HOLD ? TeachMode::ADMITTANCE
                                                            : TeachMode::RETURN_HOME;
    if (requested_teach_mode_.compare_exchange_weak(
            current, next, std::memory_order_acq_rel, std::memory_order_acquire)) {
      response->success = true;
      response->message = next == TeachMode::ADMITTANCE
                              ? "Admittance requested."
                              : "Smooth return to configured home requested.";
      return;
    }
  }
}

bool AdmittanceController::reference_state_interfaces_() {
  for (auto &state_interface : state_interfaces_) {
    if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
      joint_position_state_interfaces_.emplace_back(std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == HW_IF_SAMPLE_TIME) {
      sample_time_state_interface_ptr_ =
          std::make_unique<std::reference_wrapper<hardware_interface::LoanedStateInterface>>(
              std::ref(state_interface));
    }
    if (state_interface.get_interface_name() == HW_IF_SESSION_STATE) {
      session_state_interface_ptr_ =
          std::make_unique<std::reference_wrapper<hardware_interface::LoanedStateInterface>>(
              std::ref(state_interface));
    }
  }
  if (!estimated_ft_sensor_ptr_->assign_loaned_state_interfaces(state_interfaces_)) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Failed to assign estimated force torque state interfaces.");
    return false;
  }
  if (joint_position_state_interfaces_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint position state interfaces '%ld' does not match the number of joints "
        "in the robot '%d'.",
        joint_position_state_interfaces_.size(), lbr_fri_ros2::N_JNTS);
    return false;
  }
  return true;
}

void AdmittanceController::clear_state_interfaces_() {
  joint_position_state_interfaces_.clear();
  estimated_ft_sensor_ptr_->release_interfaces();
}

void AdmittanceController::configure_joint_names_() {
  if (joint_names_.size() != lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint names (%ld) does not match the number of joints in the robot (%d).",
        joint_names_.size(), lbr_fri_ros2::N_JNTS);
    throw std::runtime_error("Failed to configure joint names.");
  }
  std::string robot_name = this->get_node()->get_parameter("robot_name").as_string();
  for (int i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_names_[i] = robot_name + "_A" + std::to_string(i + 1);
  }
}

void AdmittanceController::configure_admittance_impl_() {
  if (this->get_node()->get_parameter("admittance.mass").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Number of mass values (%ld) does not match the number of cartesian degrees of "
                 "freedom (%d).",
                 this->get_node()->get_parameter("admittance.mass").as_double_array().size(),
                 lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure admittance parameters.");
  }
  if (this->get_node()->get_parameter("admittance.damping").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of damping values (%ld) does not match the number of cartesian degrees of freedom "
        "(%d).",
        this->get_node()->get_parameter("admittance.damping").as_double_array().size(),
        lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure admittance parameters.");
  }
  if (this->get_node()->get_parameter("admittance.stiffness").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(this->get_node()->get_logger(),
                 "Number of stiffness values (%ld) does not match the number of cartesian degrees "
                 "of freedom "
                 "(%d).",
                 this->get_node()->get_parameter("admittance.stiffness").as_double_array().size(),
                 lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure admittance parameters.");
  }
  lbr_fri_ros2::cart_array_t mass_array;
  for (unsigned int i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    mass_array[i] = this->get_node()->get_parameter("admittance.mass").as_double_array()[i];
  }
  lbr_fri_ros2::cart_array_t damping_array;
  for (unsigned int i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    damping_array[i] = this->get_node()->get_parameter("admittance.damping").as_double_array()[i];
  }
  lbr_fri_ros2::cart_array_t stiffness_array;
  for (unsigned int i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    stiffness_array[i] =
        this->get_node()->get_parameter("admittance.stiffness").as_double_array()[i];
  }
  admittance_impl_ptr_ = std::make_unique<lbr_fri_ros2::AdmittanceImpl>(
      lbr_fri_ros2::AdmittanceParameters{mass_array, damping_array, stiffness_array});
}

void AdmittanceController::configure_inv_jac_ctrl_impl_() {
  if (this->get_node()->get_parameter("inv_jac_ctrl.joint_gains").as_double_array().size() !=
      lbr_fri_ros2::N_JNTS) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of joint gains (%ld) does not match the number of joints in the robot (%d).",
        this->get_node()->get_parameter("inv_jac_ctrl.joint_gains").as_double_array().size(),
        lbr_fri_ros2::N_JNTS);
    throw std::runtime_error("Failed to configure joint gains.");
  }
  if (this->get_node()->get_parameter("inv_jac_ctrl.cartesian_gains").as_double_array().size() !=
      lbr_fri_ros2::CARTESIAN_DOF) {
    RCLCPP_ERROR(
        this->get_node()->get_logger(),
        "Number of cartesian gains (%ld) does not match the number of cartesian degrees of freedom "
        "(%d).",
        this->get_node()->get_parameter("inv_jac_ctrl.cartesian_gains").as_double_array().size(),
        lbr_fri_ros2::CARTESIAN_DOF);
    throw std::runtime_error("Failed to configure cartesian gains.");
  }
  lbr_fri_ros2::jnt_array_t joint_gains_array;
  for (unsigned int i = 0; i < lbr_fri_ros2::N_JNTS; ++i) {
    joint_gains_array[i] =
        this->get_node()->get_parameter("inv_jac_ctrl.joint_gains").as_double_array()[i];
  }
  lbr_fri_ros2::cart_array_t cartesian_gains_array;
  for (unsigned int i = 0; i < lbr_fri_ros2::CARTESIAN_DOF; ++i) {
    cartesian_gains_array[i] =
        this->get_node()->get_parameter("inv_jac_ctrl.cartesian_gains").as_double_array()[i];
  }
  inv_jac_ctrl_impl_ptr_ = std::make_unique<lbr_fri_ros2::InvJacCtrlImpl>(
      robot_description_,
      lbr_fri_ros2::InvJacCtrlParameters{
          this->get_node()->get_parameter("inv_jac_ctrl.chain_root").as_string(),
          this->get_node()->get_parameter("inv_jac_ctrl.chain_tip").as_string(),
          false, // always assume twist in root frame
          this->get_node()->get_parameter("inv_jac_ctrl.damping").as_double(),
          this->get_node()->get_parameter("inv_jac_ctrl.max_linear_velocity").as_double(),
          this->get_node()->get_parameter("inv_jac_ctrl.max_angular_velocity").as_double(),
          joint_gains_array, cartesian_gains_array});
}

void AdmittanceController::zero_all_values_() {
  f_ext_.setZero();
  delta_x_.setZero();
  dx_.setZero();
  ddx_.setZero();
  std::fill(dq_.begin(), dq_.end(), 0.0);
  twist_command_.setZero();
}

void AdmittanceController::log_info_() const {
  admittance_impl_ptr_->log_info();
  inv_jac_ctrl_impl_ptr_->log_info();
}
} // namespace lbr_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_ros2_control::AdmittanceController,
                       controller_interface::ControllerInterface)
