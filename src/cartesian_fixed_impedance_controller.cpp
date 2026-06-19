// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_human_friendly_controllers/cartesian_fixed_impedance_controller.h>

#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_human_friendly_controllers {

bool CartesianFixedImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                             ros::NodeHandle& node_handle) {
  sub_equilibrium_pose_ = node_handle.subscribe(
      "/equilibrium_pose", 20, &CartesianFixedImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  pub_cartesian_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/cartesian_pose", 1);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianFixedImpedanceController: Could not read parameter arm_id");
    return false;
  }

  auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface == nullptr) {
    ROS_ERROR("CartesianFixedImpedanceController: Could not get Cartesian Pose interface from hardware");
    return false;
  }
  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianFixedImpedanceController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianFixedImpedanceController: Could not get state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianFixedImpedanceController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianFixedImpedanceController::starting(const ros::Time& /*time*/) {
  pose_d_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  Eigen::Affine3d initial(Eigen::Matrix4d::Map(pose_d_.data()));
  std::lock_guard<std::mutex> lock(target_mutex_);
  position_d_target_ = initial.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial.rotation());
}

void CartesianFixedImpedanceController::update(const ros::Time& /*time*/,
                                               const ros::Duration& /*period*/) {
  // Publish current EE pose
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.pose.position.x = position[0];
  pose_msg.pose.position.y = position[1];
  pose_msg.pose.position.z = position[2];
  pose_msg.pose.orientation.x = orientation.x();
  pose_msg.pose.orientation.y = orientation.y();
  pose_msg.pose.orientation.z = orientation.z();
  pose_msg.pose.orientation.w = orientation.w();
  pub_cartesian_pose_.publish(pose_msg);

  // Read current commanded pose
  Eigen::Affine3d current(Eigen::Matrix4d::Map(pose_d_.data()));
  Eigen::Vector3d pos_current(current.translation());
  Eigen::Quaterniond ori_current(current.rotation());

  // Fetch target under lock
  Eigen::Vector3d pos_target;
  Eigen::Quaterniond ori_target;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    pos_target = position_d_target_;
    ori_target = orientation_d_target_;
  }

  // Low-pass filter toward target
  Eigen::Vector3d pos_new = pos_current + filter_params_ * (pos_target - pos_current);
  Eigen::Quaterniond ori_new = ori_current.slerp(filter_params_, ori_target);

  Eigen::Affine3d new_pose = Eigen::Affine3d::Identity();
  new_pose.translation() = pos_new;
  new_pose.linear() = ori_new.toRotationMatrix();
  Eigen::Map<Eigen::Matrix4d>(pose_d_.data()) = new_pose.matrix();

  cartesian_pose_handle_->setCommand(pose_d_);
}

void CartesianFixedImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() = -orientation_d_target_.coeffs();
  }
}

}  // namespace franka_human_friendly_controllers

PLUGINLIB_EXPORT_CLASS(franka_human_friendly_controllers::CartesianFixedImpedanceController,
                       controller_interface::ControllerBase)
