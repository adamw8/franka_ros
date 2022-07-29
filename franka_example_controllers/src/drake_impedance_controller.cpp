// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/drake_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

namespace franka_example_controllers {

bool DrakeImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

  sub_torque_commands_ = node_handle.subscribe(
      "/c3/franka_input", 1, &DrakeImpedanceController::torqueCommandCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  pub_joint_states_ = node_handle.advertise<sensor_msgs::JointState>("/c3/joint_states", 1);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("DrakeImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "DrakeImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DrakeImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "DrakeImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "DrakeImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "DrakeImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  /// NOTE: MAKE SURE THIS IS CONSISTENT WITH PARAMETERS.YAML IN DAIRLIB EXAMPLES
  std::array<double, 7> q_start{{0, 0.275, 0, -2.222, 0, 2.497, 0}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  tau_d_.setZero();

  return true;
}

void DrakeImpedanceController::starting(const ros::Time& time) {
  std::lock_guard<std::mutex> tau_d_mutex_lock(tau_d_mutex_);
  tau_d_.setZero();
}

void DrakeImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau(robot_state.tau_J.data());

  // local copy of tau_d_
  // allows function to release lock ASAP
  Eigen::Matrix<double, 7, 1> tau_d;
  tau_d.setZero();
  
  std::unique_lock<std::mutex> tau_d_mutex_lock(tau_d_mutex_);
  for (int i = 0; i < 7; i++){
    tau_d(i) = tau_d_(i);
  }
  tau_d_mutex_lock.unlock();

  // perform safety checks on tau_d

  // Saturate torque rate to avoid discontinuities
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // std::cout << "before clamping" << std::endl;
  // for (size_t i = 0; i < 7; ++i) {
  //   std::cout << "joint" << i+1 << ": " << tau_d(i) << std::endl;
  // }

  tau_d << clampTorques(tau_d);
  // std::cout << "after clamping" << std::endl;
  // for (size_t i = 0; i < 7; ++i) {
  //   std::cout << "joint" << i+1 << ": " << tau_d(i) << std::endl;
  // }

  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  // std::cout << "after saturating" << std::endl;
  // for (size_t i = 0; i < 7; ++i) {
  //   std::cout << "prev_joint" << i+1 << ": " << tau_J_d(i) << std::endl;
  //   std::cout << "joint" << i+1 << ": " << tau_d(i) << std::endl;
  // }

  // send the torques
  for (size_t i = 0; i < 7; ++i) {
    // std::cout << "joint" << i+1 << ": " << tau_d(i) << std::endl;
    joint_handles_[i].setCommand(tau_d(i));
    // joint_handles_[i].setCommand(0.0);
  }

  sensor_msgs::JointState msg;
  msg.position.resize(7);
  msg.velocity.resize(7);
  msg.effort.resize(7);
  for (size_t i = 0; i < 7; i++){
    msg.position[i] = q(i);
    msg.velocity[i] = dq(i);
    msg.effort[i] = tau(i);
  }
  pub_joint_states_.publish(msg);
}

Eigen::Matrix<double, 7, 1> DrakeImpedanceController::clampTorques(
    const Eigen::Matrix<double, 7, 1>& tau_d) const {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_clamped{};

  double limit = 20;
  double wrist_limit = 2;

  for (size_t i = 0; i < 4; i++) {
    tau_d_clamped(i) = tau_d(i);
    if (abs(tau_d(i)) > limit) {
      double sign = tau_d(i) / abs(tau_d(i));
      tau_d_clamped(i) = sign * limit;
    }
  }
  for (size_t i = 4; i < 7; i++) {
    tau_d_clamped(i) = tau_d(i);
    if (abs(tau_d(i)) > wrist_limit) {
      double sign = tau_d(i) / abs(tau_d(i));
      tau_d_clamped(i) = sign * wrist_limit;
    }
  }
  return tau_d_clamped;
}

Eigen::Matrix<double, 7, 1> DrakeImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void DrakeImpedanceController::torqueCommandCallback(
    const std_msgs::Float64MultiArray& msg) {
  
  std::lock_guard<std::mutex> tau_d_mutex_lock(tau_d_mutex_);

  for (int i = 0; i < 7; i++){
    tau_d_(i) = msg.data[i];
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DrakeImpedanceController,
                       controller_interface::ControllerBase)
