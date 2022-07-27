// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "std_msgs/Float64MultiArray.h"

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

class DrakeImpedanceController : public controller_interface::MultiInterfaceController<
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface,
                                                hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // should this be tau_J?
  
  std::mutex tau_d_mutex_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;

  Eigen::Matrix<double, 8, 1> tau_d_;
  const double delta_tau_max_{1.0};

  // ROS Subscriber
  ros::Subscriber sub_torque_commands_;   // read torque commands sent from drake
  void torqueCommandCallback(const std_msgs::Float64MultiArray& msg);
};

}  // namespace franka_example_controllers
