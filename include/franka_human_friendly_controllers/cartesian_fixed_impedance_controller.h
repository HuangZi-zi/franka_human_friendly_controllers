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

  // Currently commanded pose (filtered toward target)
  std::array<double, 16> pose_d_;

  // Target pose from /equilibrium_pose subscriber
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  std::mutex target_mutex_;

  const double filter_params_{0.005};

  ros::Subscriber sub_equilibrium_pose_;
  ros::Publisher pub_cartesian_pose_;

  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace franka_human_friendly_controllers
