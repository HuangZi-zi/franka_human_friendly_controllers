// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_human_friendly_controllers/cartesian_fixed_impedance_controller.h>

#include <cmath>
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
    ROS_ERROR("CartesianFixedImpedanceController: Could not get Cartesian Pose interface");
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
    ROS_ERROR("CartesianFixedImpedanceController: Could not get state interface");
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
  std::array<double, 16> initial_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  Eigen::Affine3d initial(Eigen::Matrix4d::Map(initial_pose.data()));

  pos_d_ = initial.translation();
  ori_d_ = Eigen::Quaterniond(initial.rotation());
  vel_d_.setZero();
  ang_vel_d_.setZero();

  std::lock_guard<std::mutex> lock(target_mutex_);
  pos_d_target_ = pos_d_;
  ori_d_target_ = ori_d_;
}

void CartesianFixedImpedanceController::update(const ros::Time& /*time*/,
                                               const ros::Duration& /*period*/) {
  // --- Publish current EE pose ---
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

  // --- Fetch target under lock ---
  Eigen::Vector3d pos_target;
  Eigen::Quaterniond ori_target;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    pos_target = pos_d_target_;
    ori_target = ori_d_target_;
  }

  // --- Translational velocity-limited motion ---
  // Desired velocity: point toward target, capped by max_vel_ and braking distance
  Eigen::Vector3d pos_error = pos_target - pos_d_;
  double dist = pos_error.norm();
  Eigen::Vector3d desired_vel;
  if (dist < 1e-6) {
    desired_vel.setZero();
  } else {
    // Braking: decelerate so we stop exactly at the target
    double brake_speed = std::sqrt(2.0 * max_acc_ * dist);
    double speed = std::min(brake_speed, max_vel_);
    desired_vel = pos_error.normalized() * speed;
  }
  // Limit acceleration: cap velocity change per cycle
  Eigen::Vector3d vel_diff = desired_vel - vel_d_;
  double vel_diff_norm = vel_diff.norm();
  double max_vel_change = max_acc_ * dt_;
  if (vel_diff_norm > max_vel_change) {
    vel_d_ += vel_diff * (max_vel_change / vel_diff_norm);
  } else {
    vel_d_ = desired_vel;
  }
  pos_d_ += vel_d_ * dt_;

  // --- Rotational velocity-limited motion ---
  // Error quaternion from current to target
  Eigen::Quaterniond error_quat = ori_target * ori_d_.inverse();
  if (error_quat.w() < 0.0) {
    error_quat.coeffs() = -error_quat.coeffs();
  }
  Eigen::AngleAxisd error_aa(error_quat);
  double angular_dist = error_aa.angle();

  Eigen::Vector3d desired_ang_vel;
  if (angular_dist < 1e-6) {
    desired_ang_vel.setZero();
  } else {
    double brake_ang_speed = std::sqrt(2.0 * max_rotational_acc_ * angular_dist);
    double ang_speed = std::min(brake_ang_speed, max_rotational_vel_);
    desired_ang_vel = error_aa.axis() * ang_speed;
  }
  // Limit angular acceleration
  Eigen::Vector3d ang_vel_diff = desired_ang_vel - ang_vel_d_;
  double ang_vel_diff_norm = ang_vel_diff.norm();
  double max_ang_vel_change = max_rotational_acc_ * dt_;
  if (ang_vel_diff_norm > max_ang_vel_change) {
    ang_vel_d_ += ang_vel_diff * (max_ang_vel_change / ang_vel_diff_norm);
  } else {
    ang_vel_d_ = desired_ang_vel;
  }
  // Integrate angular velocity into orientation
  double ang_vel_norm = ang_vel_d_.norm();
  if (ang_vel_norm > 1e-10) {
    Eigen::Quaterniond delta_rot(
        Eigen::AngleAxisd(ang_vel_norm * dt_, ang_vel_d_.normalized()));
    ori_d_ = (delta_rot * ori_d_).normalized();
  }

  // --- Build 4x4 command and send ---
  Eigen::Affine3d cmd = Eigen::Affine3d::Identity();
  cmd.translation() = pos_d_;
  cmd.linear() = ori_d_.toRotationMatrix();

  std::array<double, 16> cmd_array;
  Eigen::Map<Eigen::Matrix4d>(cmd_array.data()) = cmd.matrix();
  cartesian_pose_handle_->setCommand(cmd_array);
}

void CartesianFixedImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  pos_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last(ori_d_target_);
  ori_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  // Ensure shortest arc
  if (last.coeffs().dot(ori_d_target_.coeffs()) < 0.0) {
    ori_d_target_.coeffs() = -ori_d_target_.coeffs();
  }
}

}  // namespace franka_human_friendly_controllers

PLUGINLIB_EXPORT_CLASS(franka_human_friendly_controllers::CartesianFixedImpedanceController,
                       controller_interface::ControllerBase)
