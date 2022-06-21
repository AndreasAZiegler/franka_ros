// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <balance_controller/balance_controller_cartesian.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <cmath>

namespace balance_controller {

bool BalanceControllerCartesian::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "BalanceControllerCartesian: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("BalanceControllerCartesian: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "BalanceControllerCartesian: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("BalanceControllerCartesian: Could not get state interface from hardware");
    return false;
  }

  /*
  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "BalanceControllerCartesian: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "BalanceControllerCartesian: Exception getting state handle: " << e.what());
    return false;
  }
  */

  steps_ = 50;
  current_waypoint_ = 0;

  return true;
}

void BalanceControllerCartesian::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);

  ROS_INFO_STREAM("initial_pose_:");
  for (const auto & pose : initial_pose_) {
    ROS_INFO_STREAM(pose << " ");
  }

  /*
  auto inertia_matrix = cartesian_pose_handle_->getRobotState().I_ee;
  ROS_INFO_STREAM("inertia_matrix:");
  for (const auto & item : inertia_matrix) {
    ROS_INFO_STREAM(item << " ");
  }

  auto center_of_mass = cartesian_pose_handle_->getRobotState().F_x_Cee;
  ROS_INFO_STREAM("center_of_mass:");
  for (const auto & item : center_of_mass) {
    ROS_INFO_STREAM(item << " ");
  }
  */

  cartesian_pose_handle_->setCommand(initial_pose_);

  double radius = 0.3;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);
  //new_pose[12] -= delta_x;
  //new_pose[13] -= delta_y;
  //new_pose[14] += delta_z;

  Eigen::Transform<double, 3, Eigen::Affine> T_current;
  T_current.matrix() << initial_pose_[0], initial_pose_[4], initial_pose_[8], initial_pose_[12],
                        initial_pose_[1], initial_pose_[5], initial_pose_[9], initial_pose_[13],
                        initial_pose_[2], initial_pose_[6], initial_pose_[10], initial_pose_[14],
                        initial_pose_[3], initial_pose_[7], initial_pose_[11], initial_pose_[15];
  //ROS_INFO_STREAM("T_current:\n" << T_current.matrix());

  double degree = 1.0 * M_PI / 180;
  double cos_value = cos(degree);
  double sin_value = sin(degree);
  Eigen::Transform<double, 3, Eigen::Affine> T_rotation;
  T_rotation.matrix() << 1, 0, 0, 0, 0, cos_value, -sin_value, 0, 0, sin_value, cos_value, 0, 0, 0, 0, 1;
  //ROS_INFO_STREAM("T_rotation: " << T_rotation.matrix());

  Eigen::Transform<double, 3, Eigen::Affine> T_result = T_rotation * T_current;
  /*
  double angle_x = 2.92932 + degree;
  double angle_y = -1.61303 + 0;
  double angle_z = 1.00276;
  T_result.matrix() << cos(angle_y) * cos(angle_z), cos(angle_z) * sin(angle_x) * sin(angle_y) - cos(angle_x) * sin(angle_z), sin(angle_x) * sin(angle_z) + cos(angle_x) * cos(angle_z) * sin(angle_y), 0.455991,
                       cos(angle_y) * sin(angle_z), cos(angle_x) * cos(angle_z) + sin(angle_x) * sin(angle_y) * sin(angle_z), cos(angle_x) * sin(angle_y) * sin(angle_z) - cos(angle_z) * sin(angle_x), 0.421773,
                       -sin(angle_y), cos(angle_y) * sin(angle_x), cos(angle_x) * cos(angle_y), 0.889305;
   */
  //ROS_INFO_STREAM("T_result:\n" << T_result.matrix());

  const auto& result_matrix = T_result.matrix();
  std::array<double, 16> new_pose = {result_matrix(0), result_matrix(1), result_matrix(2), result_matrix(3),
                                     result_matrix(4), result_matrix(5), result_matrix(6), result_matrix(7),
                                     result_matrix(8), result_matrix(9), result_matrix(10), result_matrix(11),
                                     result_matrix(12), result_matrix(13), result_matrix(14), result_matrix(15)};
  
  Eigen::Matrix<double, 3> difference = result_matrix - T_current.matrix();

  std::size_t steps = 50;
  waypoints_.clear();
  for (std::size_t i = 0; i < steps; ++i) {
    waypoints_.emplace_back{i / steps * difference};
  }
}

void BalanceControllerCartesian::update(const ros::Time& time,
                                            const ros::Duration& period) {
  elapsed_time_ += period;
  //new_pose = initial_pose_;
  
  if (current_waypoint_ < steps_) {
    const auto& result_matrix = waypoints_[current_waypoint_];
    std::array<double, 16> new_pose = {result_matrix(0), result_matrix(1), result_matrix(2), result_matrix(3),
                                       result_matrix(4), result_matrix(5), result_matrix(6), result_matrix(7),
                                       result_matrix(8), result_matrix(9), result_matrix(10), result_matrix(11),
                                       result_matrix(12), result_matrix(13), result_matrix(14), result_matrix(15)};

    cartesian_pose_handle_->setCommand(new_pose);
    current_waypoint_++;
  }

  //ROS_INFO_STREAM("T_result.matrix():\n" << T_result.matrix());
  Eigen::Matrix3d rot_mat;
  rot_mat << T_current.matrix()(0, 0), T_current.matrix()(0, 1), T_current.matrix()(0, 2),
             T_current.matrix()(1, 0), T_current.matrix()(1, 1), T_current.matrix()(1, 2),
             T_current.matrix()(2, 0), T_current.matrix()(2, 1), T_current.matrix()(2, 2);
  //ROS_INFO_STREAM("rot_mat:\n" << rot_mat);
  /*
  Eigen::Vector3d euler_angles = rot_mat.eulerAngles(2, 1, 0);
  ROS_INFO_STREAM("euler_angles:\n" << euler_angles);
  */

  /*
  ROS_INFO_STREAM("T_result:\n" << T_result.matrix());
  ROS_INFO_STREAM("new_pose:");
  for (const auto& element : new_pose) {
    ROS_INFO_STREAM(element);
  }
  ROS_INFO_STREAM("Differences:");
  for (std::size_t i = 0; i < new_pose.size(); ++i) {
    ROS_INFO_STREAM(i << ": " << new_pose[i] - initial_pose_[i]);
  }
  */
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceControllerCartesian,
                       controller_interface::ControllerBase)
