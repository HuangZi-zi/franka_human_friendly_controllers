// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

namespace franka_human_friendly_controllers {

class CartesianFixedImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaPoseCartesianInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

  // Current commanded pose
  Eigen::Vector3d pos_d_;
  Eigen::Quaterniond ori_d_;

  // Current commanded velocity (used to enforce acc limits → no velocity jumps)
  Eigen::Vector3d vel_d_;
  Eigen::Vector3d ang_vel_d_;

  // Target from /equilibrium_pose (raw, written by callback)
  Eigen::Vector3d pos_d_target_;
  Eigen::Quaterniond ori_d_target_;
  std::mutex target_mutex_;

  // Filtered targets — attract toward raw targets with a 1st-order low-pass,
  // eliminating velocity discontinuities when the commanded pose jumps.
  Eigen::Vector3d pos_d_target_filtered_;
  Eigen::Quaterniond ori_d_target_filtered_;

  // Smoothing low-pass filter coefficient: alpha = 1 - exp(-2*pi*f_smooth*dt)
  double alpha_{0.0};
  double smoothing_frequency_{5.0};  // Hz cutoff (tunable via ROS param)

  // Motion limits (conservative, well within Franka hardware limits)
  const double max_vel_{0.1};             // m/s
  const double max_acc_{1.0};             // m/s²
  const double max_rotational_vel_{0.2};  // rad/s
  const double max_rotational_acc_{2.0};  // rad/s²
  const double dt_{0.001};               // 1 kHz control rate

  ros::Subscriber sub_equilibrium_pose_;
  ros::Publisher pub_cartesian_pose_;

  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace franka_human_friendly_controllers
