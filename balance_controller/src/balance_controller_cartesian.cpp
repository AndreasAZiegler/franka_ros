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

  steps_ = 4000;
  current_waypoint_ = 0;

  return true;
}

void BalanceControllerCartesian::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  /*
  ROS_INFO_STREAM("initial_pose_:");
  for (const auto& pose : initial_pose_) {
    ROS_INFO_STREAM(pose << " ");
  }
  */

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

  // cartesian_pose_handle_->setCommand(initial_pose_);

  /*
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);
  new_pose[12] -= delta_x;
  new_pose[13] -= delta_y;
  new_pose[14] += delta_z;
  */

  // Eigen::Transform<double, 3, Eigen::Affine> T_initial;
  T_initial_.matrix() << initial_pose_[0], initial_pose_[4], initial_pose_[8], initial_pose_[12],
      initial_pose_[1], initial_pose_[5], initial_pose_[9], initial_pose_[13], initial_pose_[2],
      initial_pose_[6], initial_pose_[10], initial_pose_[14], initial_pose_[3], initial_pose_[7],
      initial_pose_[11], initial_pose_[15];
  //ROS_INFO_STREAM("T_initial_:\n" << T_initial_.matrix());

  // Eigen::Transform<double, 3, Eigen::Affine> T_ref = T_initial_;
  T_ref_ = T_initial_;
  T_ref_.translation() << 0.0, 0.0, 0.0;
  //ROS_INFO_STREAM("T_ref_:\n" << T_ref_.matrix());

  // Eigen::Transform<double, 3, Eigen::Affine> T_offset;
  T_offset_.linear().setIdentity();
  T_offset_.translation() = T_initial_.translation();
  //ROS_INFO_STREAM("T_offset_:\n" << T_offset_.matrix());

  double rad = 5.0 /*deg*/ * M_PI / 180;
  Eigen::Vector3d euler_angles = T_initial_.linear().eulerAngles(2, 1, 0);

  Eigen::Matrix3d rot_mat;
  rot_mat = Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  Eigen::Transform<double, 3, Eigen::Affine> T_rotation;
  T_rotation.linear() = rot_mat;
  T_rotation.translation() << 0.0, 0.0, 0.0;
  //ROS_INFO_STREAM("T_rotation:\n" << T_rotation.matrix());
  Eigen::Transform<double, 3, Eigen::Affine> T_end = T_offset_ * T_rotation * T_ref_;
  //ROS_INFO_STREAM("T_end:\n" << T_end.matrix());
  //ROS_INFO_STREAM("Difference:\n" << T_end.matrix() - T_initial_.matrix());

  // Constraints
  double x_q_0 = euler_angles(2);
  double y_q_0 = euler_angles(1);

  double x_q_1 = x_q_0 + rad;
  double y_q_1 = y_q_0;

  double x_v_0 = 0;
  double y_v_0 = 0;

  double x_v_1 = 0;
  double y_v_1 = 0;

  double x_a_0 = 0;
  double y_a_0 = 0;

  double x_a_1 = 0;
  double y_a_1 = 0;

  Eigen::Matrix<double, 6, 6> A;
  Eigen::Matrix<double, 6, 1> constraints;
  A << 0, 0, 0, 0, 0, 1,
       1, 1, 1, 1, 1, 1,
       0, 0, 0, 0, 1, 0,
       5, 4, 3, 2, 1, 0,
       0, 0, 0, 2, 0, 0,
       20, 12, 6, 2, 0, 0;
  constraints << x_q_0, x_q_1, x_v_0, x_v_1, x_a_0, x_a_1;
  Eigen::Matrix<double, 6, 1> x_params = A.colPivHouseholderQr().solve(constraints);
  const double& x_a = x_params(0);
  const double& x_b = x_params(1);
  const double& x_c = x_params(2);
  const double& x_d = x_params(3);
  const double& x_e = x_params(4);
  const double& x_f = x_params(5);

  double previous_x_value = 0.0;
  double alpha = 1.0;
  waypoints_.clear();
  for (std::size_t i = 0; i < steps_ + 1; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(steps_);
    // ROS_INFO_STREAM("t: " << t);
    double x_value = x_f + x_e * t + x_d * t * t + x_c * t * t * t + x_b * t * t * t * t +
                     x_a * t * t * t * t * t;
    x_value -= x_q_0;
    x_value = (1 - alpha) * previous_x_value + alpha * x_value;
    previous_x_value = x_value;
    Eigen::Matrix3d rot_mat;
    rot_mat = Eigen::AngleAxisd(x_value, Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Transform<double, 3, Eigen::Affine> T_rotation;
    T_rotation.linear() = rot_mat;
    T_rotation.translation() << 0.0, 0.0, 0.0;
    Eigen::Transform<double, 3, Eigen::Affine> T_waypoint = T_offset_ * T_rotation * T_ref_;

    // ROS_INFO_STREAM("Waypoint " << i << ":\n" << T_waypoint.matrix());
    //ROS_INFO_STREAM("Difference:\n" << T_waypoint.matrix() - T_initial_.matrix());

    waypoints_.emplace_back(T_waypoint.matrix());
  }

  /*
  double cos_value = cos(degree);
  double sin_value = sin(degree);
  Eigen::Transform<double, 3, Eigen::Affine> T_rotation;
  T_rotation.matrix() << 1, 0, 0, 0, 0, cos_value, -sin_value, 0, 0, sin_value, cos_value, 0, 0, 0,
  0, 1;
  //ROS_INFO_STREAM("T_rotation: " << T_rotation.matrix());

  Eigen::Transform<double, 3, Eigen::Affine> T_result = T_rotation * T_current;
  */

  /*
  double angle_x = 2.92932 + degree;
  double angle_y = -1.61303 + 0;
  double angle_z = 1.00276;
  T_result.matrix() << cos(angle_y) * cos(angle_z), cos(angle_z) * sin(angle_x) * sin(angle_y) -
  cos(angle_x) * sin(angle_z), sin(angle_x) * sin(angle_z) + cos(angle_x) * cos(angle_z) *
  sin(angle_y), 0.455991, cos(angle_y) * sin(angle_z), cos(angle_x) * cos(angle_z) + sin(angle_x) *
  sin(angle_y) * sin(angle_z), cos(angle_x) * sin(angle_y) * sin(angle_z) - cos(angle_z) *
  sin(angle_x), 0.421773, -sin(angle_y), cos(angle_y) * sin(angle_x), cos(angle_x) * cos(angle_y),
  0.889305;
   */
  // ROS_INFO_STREAM("T_result:\n" << T_result.matrix());

  /*
  const auto& result_matrix = T_result.matrix();
  std::array<double, 16> new_pose = {result_matrix(0), result_matrix(1), result_matrix(2),
  result_matrix(3), result_matrix(4), result_matrix(5), result_matrix(6), result_matrix(7),
                                     result_matrix(8), result_matrix(9), result_matrix(10),
  result_matrix(11), result_matrix(12), result_matrix(13), result_matrix(14), result_matrix(15)};

  Eigen::Matrix<double, 3> difference = result_matrix - T_current.matrix();

  waypoints_.clear();
  for (std::size_t i = 0; i < steps; ++i) {
    waypoints_.emplace_back{i / steps * difference};
  }
  */
  elapsed_time_ = ros::Duration(0.0);
}

void BalanceControllerCartesian::update(const ros::Time& /*time*/, const ros::Duration& period) {
  elapsed_time_ += period;
  // new_pose = initial_pose_;

  if (current_waypoint_ < steps_ + 1) {
    const auto& result_matrix = waypoints_.at(current_waypoint_);
    std::array<double, 16> new_pose = {
        result_matrix(0),  result_matrix(1),  result_matrix(2),  result_matrix(3),
        result_matrix(4),  result_matrix(5),  result_matrix(6),  result_matrix(7),
        result_matrix(8),  result_matrix(9),  result_matrix(10), result_matrix(11),
        result_matrix(12), result_matrix(13), result_matrix(14), result_matrix(15)};

    cartesian_pose_handle_->setCommand(new_pose);
    current_waypoint_++;
  } else if (current_waypoint_ == steps_ + 1) {
    ROS_INFO_STREAM("Goal reached!");
    current_waypoint_++;
  }

  /*
  double radius = 0.3;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);
  // std::array<double, 16> new_pose = initial_pose_;
  // new_pose[12] -= delta_x;
  // new_pose[14] -= delta_z;
  Eigen::Vector3d euler_angles = T_initial_.linear().eulerAngles(2, 1, 0);

  Eigen::Matrix3d rot_mat;
  rot_mat = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  Eigen::Transform<double, 3, Eigen::Affine> T_rotation;
  T_rotation.linear() = rot_mat;
  T_rotation.translation() << 0.0, 0.0, 0.0;
  // ROS_INFO_STREAM("T_rotation:\n" << T_rotation.matrix());
  Eigen::Transform<double, 3, Eigen::Affine> T_waypoint = T_offset_ * T_rotation * T_ref_;

  const auto& result_matrix = T_waypoint.matrix();
  std::array<double, 16> new_pose = {
      result_matrix(0),  result_matrix(1),  result_matrix(2),  result_matrix(3),
      result_matrix(4),  result_matrix(5),  result_matrix(6),  result_matrix(7),
      result_matrix(8),  result_matrix(9),  result_matrix(10), result_matrix(11),
      result_matrix(12), result_matrix(13), result_matrix(14), result_matrix(15)};
  cartesian_pose_handle_->setCommand(new_pose);
  */

  // ROS_INFO_STREAM("T_result.matrix():\n" << T_result.matrix());
  /*
  Eigen::Matrix3d rot_mat;
  rot_mat << T_current.matrix()(0, 0), T_current.matrix()(0, 1), T_current.matrix()(0, 2),
             T_current.matrix()(1, 0), T_current.matrix()(1, 1), T_current.matrix()(1, 2),
             T_current.matrix()(2, 0), T_current.matrix()(2, 1), T_current.matrix()(2, 2);
  //ROS_INFO_STREAM("rot_mat:\n" << rot_mat);
  */
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

}  // namespace balance_controller

PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceControllerCartesian,
                       controller_interface::ControllerBase)
