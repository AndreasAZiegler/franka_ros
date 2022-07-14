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

double constrainAngle(double x) {
  x = fmod(x, 2 * M_PI);
  if (x < 0) {
    x += 2 * M_PI;
  }
  return x;
}

bool BalanceControllerCartesian::init(hardware_interface::RobotHW* robot_hardware,
                                      ros::NodeHandle& node_handle) {
  control_position_ = false;
  position_initialized_ = false;

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

  control_subscriber_ =
      node_handle.subscribe("control", 10, &BalanceControllerCartesian::controllCallback, this);
  position_x_subscriber_ =
      node_handle.subscribe("position_x", 10, &BalanceControllerCartesian::positionXCallback, this);
  position_y_subscriber_ =
      node_handle.subscribe("position_y", 10, &BalanceControllerCartesian::positionYCallback, this);

  position_x_ = 0.0;
  position_y_ = 0.0;

  return true;
}

void BalanceControllerCartesian::starting(const ros::Time& /* time */) {
  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  /*
  auto end_effector_frame = cartesian_pose_handle_->getRobotState().F_T_EE;
  Eigen::Transform<double, 3, Eigen::Affine> T_end_effector_frame;
  T_end_effector_frame.matrix() << end_effector_frame[0], end_effector_frame[4],
      end_effector_frame[8], end_effector_frame[12], end_effector_frame[1], end_effector_frame[5],
      end_effector_frame[9], end_effector_frame[13], end_effector_frame[2], end_effector_frame[6],
      end_effector_frame[10], end_effector_frame[14], end_effector_frame[3], end_effector_frame[7],
      end_effector_frame[11], end_effector_frame[15];
  ROS_INFO_STREAM("T_end_effector_frame:\n" << T_end_effector_frame.matrix());

  auto nominal_end_effector_frame = cartesian_pose_handle_->getRobotState().F_T_NE;
  Eigen::Transform<double, 3, Eigen::Affine> T_nominal_end_effector_frame;
  T_nominal_end_effector_frame.matrix() << nominal_end_effector_frame[0],
      nominal_end_effector_frame[4], nominal_end_effector_frame[8], nominal_end_effector_frame[12],
      nominal_end_effector_frame[1], nominal_end_effector_frame[5], nominal_end_effector_frame[9],
      nominal_end_effector_frame[13], nominal_end_effector_frame[2], nominal_end_effector_frame[6],
      nominal_end_effector_frame[10], nominal_end_effector_frame[14], nominal_end_effector_frame[3],
      nominal_end_effector_frame[7], nominal_end_effector_frame[11], nominal_end_effector_frame[15];
  ROS_INFO_STREAM("T_nominal_end_effector_frame:\n" << T_nominal_end_effector_frame.matrix());
  */

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

  T_base_end_current_.matrix() << current_pose_[0], current_pose_[4], current_pose_[8],
      current_pose_[12], current_pose_[1], current_pose_[5], current_pose_[9], current_pose_[13],
      current_pose_[2], current_pose_[6], current_pose_[10], current_pose_[14], current_pose_[3],
      current_pose_[7], current_pose_[11], current_pose_[15];
  ROS_INFO_STREAM("T_base_end_current_:\n" << T_base_end_current_.matrix());

  T_base_end_ref_ = T_base_end_current_;
  T_base_end_ref_.translation() << 0.0, 0.0, 0.0;
  // ROS_INFO_STREAM("T_ref_:\n" << T_ref_.matrix());

  T_offset_.linear().setIdentity();
  T_offset_.translation() = T_base_end_current_.translation();
  // ROS_INFO_STREAM("T_offset_:\n" << T_offset_.matrix());

  Eigen::Vector3d euler_angles = T_base_end_current_.linear().eulerAngles(2, 1, 0);
  ROS_INFO_STREAM("euler angles: x: " << constrainAngle(euler_angles(2)) * 180 / M_PI
                                      << ", y: " << constrainAngle(euler_angles(1)) * 180 / M_PI
                                      << ", z: " << constrainAngle(euler_angles(0)) * 180 / M_PI);

  elapsed_time_ = ros::Duration(0.0);
}

void BalanceControllerCartesian::update(const ros::Time& time, const ros::Duration& period) {
  // current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  const auto& robot_state = cartesian_pose_handle_->getRobotState();
  current_pose_ = robot_state.O_T_EE_c;
  auto dq = robot_state.dq;
  auto dq_d = robot_state.dq_d;
  auto ddq_d = robot_state.ddq_d;

  elapsed_time_ += period;

  double elapsed_time_double =
      (static_cast<double>(elapsed_time_.sec) + static_cast<double>(elapsed_time_.nsec * 1e-9));
  double t = elapsed_time_double / t_end_;

  if (control_position_ && position_initialized_) {
    if (t <= 1.0) {
      // ROS_INFO_STREAM("elapsed_time_double: " << elapsed_time_double << ", t: " << t);
      double x_value = x_f_ + x_e_ * t + x_d_ * t * t + x_c_ * t * t * t + x_b_ * t * t * t * t +
                       x_a_ * t * t * t * t * t;
      double x_velocity =
          5 * x_a_ * t * t * t * t + 4 * x_b_ * t * t * t + 3 * x_c_ * t * t + 2 * x_d_ * t + x_e_;
      double x_acceleration = 20 * x_a_ * t * t * t + 12 * x_b_ * t * t + 6 * x_c_ * t + 2 * x_d_;
      double y_value = y_f_ + y_e_ * t + y_d_ * t * t + y_c_ * t * t * t + y_b_ * t * t * t * t +
                       y_a_ * t * t * t * t * t;
      double y_velocity =
          5 * y_a_ * t * t * t * t + 4 * y_b_ * t * t * t + 3 * y_c_ * t * t + 2 * y_d_ * t + y_e_;
      double y_acceleration = 20 * y_a_ * t * t * t + 12 * y_b_ * t * t + 6 * y_c_ * t + 2 * y_d_;
      x_value -= x_q_0_;
      y_value -= y_q_0_;

      /*
      measured_joint_velocity_.emplace_back(dq.at(5));
      double dt = static_cast<double>(period.sec) + static_cast<double>(period.nsec * 1e-9);
      periods_.emplace_back(dt);
      x_values_.emplace_back(x_value);
      y_values_.emplace_back(y_value);
      x_velocity_.emplace_back(x_velocity);
      y_velocity_.emplace_back(y_velocity);
      x_velocity_delta_.emplace_back((x_velocity - dq_d.at(5)) / dt);
      y_velocity_delta_.emplace_back((y_velocity - dq_d.at(6)) / dt);
      x_velocity_deviation_.emplace_back((previous_x_velocity_ - dq_d.at(5)));
      y_velocity_deviation_.emplace_back((previous_y_velocity_ - dq_d.at(6)));
      x_acceleration_.emplace_back(x_acceleration);
      y_acceleration_.emplace_back(y_acceleration);
      x_acceleration_delta_.emplace_back((x_acceleration - ddq_d.at(5)) / dt);
      y_acceleration_delta_.emplace_back((y_acceleration - ddq_d.at(6)) / dt);
      x_acceleration_deviation_.emplace_back((previous_x_acceleration_ - ddq_d.at(5)));
      y_acceleration_deviation_.emplace_back((previous_y_acceleration_ - ddq_d.at(6)));

      previous_x_value_ = x_value;
      previous_y_value_ = y_value;
      previous_x_velocity_ = x_velocity;
      previous_y_velocity_ = y_velocity;
      previous_x_acceleration_ = x_acceleration;
      previous_y_acceleration_ = y_acceleration;
      */

      // Low-pass filter
      // x_value = (1 - alpha) * previous_x_value + alpha * x_value;
      // previous_x_value = x_value;

      Eigen::Matrix3d rot_mat;
      rot_mat = Eigen::AngleAxisd(x_value, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(-y_value, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
      Eigen::Transform<double, 3, Eigen::Affine> T_rotation;
      T_rotation.linear() = rot_mat;
      T_rotation.translation() << 0.0, 0.0, 0.0;

      // Eigen::Transform<double, 3, Eigen::Affine> T_waypoint = T_offset_ * T_rotation * T_ref_;
      Eigen::Transform<double, 3, Eigen::Affine> T_waypoint = T_base_end_ref_ * T_rotation;

      const auto& waypoint_matrix = T_waypoint.matrix();

      std::array<double, 16> waypoint = {
          waypoint_matrix(0),  waypoint_matrix(1),  waypoint_matrix(2),  waypoint_matrix(3),
          waypoint_matrix(4),  waypoint_matrix(5),  waypoint_matrix(6),  waypoint_matrix(7),
          waypoint_matrix(8),  waypoint_matrix(9),  waypoint_matrix(10), waypoint_matrix(11),
          waypoint_matrix(12), waypoint_matrix(13), waypoint_matrix(14), waypoint_matrix(15)};

      cartesian_pose_handle_->setCommand(waypoint);

      // publishTargetState(T_waypoint);
    } else {
      position_initialized_ = false;
      /*
      ROS_WARN_STREAM("periods_ [ms]:");
      for (const auto& value : periods_) {
        std::cerr << value * 1e3 << ", ";
      }
      std::cerr << std::endl;

      ROS_WARN_STREAM("x_velocity_deviation_:");
      for (const auto& x_value : x_velocity_deviation_) {
        std::cerr << x_value << ", ";
      }
      std::cerr << std::endl;
      ROS_WARN_STREAM("x_velocity_delta_:");
      for (const auto& x_value : x_velocity_delta_) {
        std::cerr << x_value << ", ";
      }
      std::cerr << std::endl;
      ROS_WARN_STREAM("x_acceleration_deviation_:");
      for (const auto& x_value : x_acceleration_deviation_) {
        std::cerr << x_value << ", ";
      }
      std::cerr << std::endl;
      ROS_WARN_STREAM("x_acceleration_delta_:");
      for (const auto& x_value : x_acceleration_delta_) {
        std::cerr << x_value << ", ";
      }
      std::cerr << std::endl;

      ROS_WARN_STREAM("y_velocity_deviation_:");
      for (const auto& y_value : y_velocity_deviation_) {
        std::cerr << y_value << ", ";
      }
      std::cerr << std::endl;
      ROS_WARN_STREAM("y_velocity_delta_:");
      for (const auto& y_value : y_velocity_delta_) {
        std::cerr << y_value << ", ";
      }
      std::cerr << std::endl;
      ROS_WARN_STREAM("y_acceleration_deviation_:");
      for (const auto& y_value : y_acceleration_deviation_) {
        std::cerr << y_value << ", ";
      }
      std::cerr << std::endl;
      ROS_WARN_STREAM("y_acceleration_delta_:");
      for (const auto& y_value : y_acceleration_delta_) {
        std::cerr << y_value << ", ";
      }
      std::cerr << std::endl;
      */

      T_base_end_current_.matrix() << current_pose_[0], current_pose_[4], current_pose_[8],
          current_pose_[12], current_pose_[1], current_pose_[5], current_pose_[9],
          current_pose_[13], current_pose_[2], current_pose_[6], current_pose_[10],
          current_pose_[14], current_pose_[3], current_pose_[7], current_pose_[11],
          current_pose_[15];

      Eigen::Vector3d euler_angles = T_base_end_current_.linear().eulerAngles(2, 1, 0);
      ROS_INFO_STREAM("euler angles: x: " << euler_angles(2) * 180 / M_PI
                                          << ", y: " << euler_angles(1) * 180 / M_PI
                                          << ", z: " << euler_angles(0) * 180 / M_PI);

      double normalized_x_angle = constrainAngle(euler_angles(2));
      double normalized_y_angle = constrainAngle(euler_angles(1));
      ROS_INFO_STREAM("normalized angles x: " << normalized_x_angle * 180 / M_PI
                                              << ", y: " << normalized_y_angle * 180 / M_PI);
    }
  } else {
    cartesian_pose_handle_->setCommand(current_pose_);
  }

  // publishCurrentState();
}

void BalanceControllerCartesian::publishTargetState(
    const Eigen::Transform<double, 3, Eigen::Affine>& target_pose) {
  Eigen::Quaterniond quaternion(target_pose.matrix().block<3, 3>(0, 0));

  if (target_pose_publisher_.trylock()) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_pose_publisher_.msg_.translation.x = target_pose.translation().x();
    target_pose_publisher_.msg_.translation.y = target_pose.translation().y();
    target_pose_publisher_.msg_.translation.z = target_pose.translation().z();
    target_pose_publisher_.msg_.rotation.x = quaternion.x();
    target_pose_publisher_.msg_.rotation.y = quaternion.y();
    target_pose_publisher_.msg_.rotation.z = quaternion.z();
    target_pose_publisher_.msg_.rotation.w = quaternion.w();
    target_pose_publisher_.unlockAndPublish();
  }
}

void BalanceControllerCartesian::publishCurrentState() {
  Eigen::Matrix3d rot_mat;
  rot_mat << current_pose_.at(0), current_pose_.at(4), current_pose_.at(8), current_pose_.at(1),
      current_pose_.at(5), current_pose_.at(9), current_pose_.at(2), current_pose_.at(6),
      current_pose_.at(10);
  Eigen::Quaterniond quaternion(rot_mat);

  if (current_pose_publisher_.trylock()) {
    std::lock_guard<std::mutex> lock(current_mutex_);
    current_pose_publisher_.msg_.translation.x = current_pose_.at(12);
    current_pose_publisher_.msg_.translation.y = current_pose_.at(13);
    current_pose_publisher_.msg_.translation.z = current_pose_.at(14);
    current_pose_publisher_.msg_.rotation.x = quaternion.x();
    current_pose_publisher_.msg_.rotation.y = quaternion.y();
    current_pose_publisher_.msg_.rotation.z = quaternion.z();
    current_pose_publisher_.msg_.rotation.w = quaternion.w();
    current_pose_publisher_.unlockAndPublish();
  }
}

double capValue(double value) {
  if (fabs(value) < 1e-4) {
    return 0.0;
  } else {
    return value;
  }
}

void BalanceControllerCartesian::goToPosition() {
  // current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_c;

  T_base_end_current_.matrix() << current_pose_[0], current_pose_[4], current_pose_[8],
      current_pose_[12], current_pose_[1], current_pose_[5], current_pose_[9], current_pose_[13],
      current_pose_[2], current_pose_[6], current_pose_[10], current_pose_[14], current_pose_[3],
      current_pose_[7], current_pose_[11], current_pose_[15];
  ROS_INFO_STREAM("T_base_end_current_:\n" << T_base_end_current_.matrix());

  T_base_end_ref_ = T_base_end_current_;
  // T_ref_.translation() << 0.0, 0.0, 0.0;
  // ROS_INFO_STREAM("T_ref_:\n" << T_ref_.matrix());

  T_offset_.linear().setIdentity();
  T_offset_.translation() = T_base_end_current_.translation();
  // ROS_INFO_STREAM("T_offset_:\n" << T_offset_.matrix());

  Eigen::Vector3d euler_angles = T_base_end_current_.linear().eulerAngles(2, 1, 0);
  ROS_INFO_STREAM("euler angles: x: " << euler_angles(2) * 180 / M_PI
                                      << ", y: " << euler_angles(1) * 180 / M_PI
                                      << ", z: " << euler_angles(0) * 180 / M_PI);

  Eigen::Matrix3d rot_mat;
  double normalized_x_angle = constrainAngle(euler_angles(2));
  double normalized_y_angle = constrainAngle(euler_angles(1));
  ROS_INFO_STREAM("normalized angles x: " << normalized_x_angle * 180 / M_PI
                                          << ", y: " << normalized_y_angle * 180 / M_PI);
  double x_diff = position_x_ - normalized_x_angle;
  double y_diff = position_y_ - normalized_y_angle;
  ROS_INFO_STREAM("x_diff: " << x_diff * 180 / M_PI);
  ROS_INFO_STREAM("y_diff: " << y_diff * 180 / M_PI);

  rot_mat = Eigen::AngleAxisd(x_diff, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(y_diff, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  Eigen::Transform<double, 3, Eigen::Affine> T_rotation;
  T_rotation.linear() = rot_mat;
  T_rotation.translation() << 0.0, 0.0, 0.0;
  // ROS_INFO_STREAM("T_rotation:\n" << T_rotation.matrix());
  // Eigen::Transform<double, 3, Eigen::Affine> T_end = T_offset_ * T_rotation * T_ref_;
  Eigen::Transform<double, 3, Eigen::Affine> T_end = T_base_end_ref_ * T_rotation;
  // ROS_INFO_STREAM("T_end:\n" << T_end.matrix());

  double x_max_acceleration = 0.0;
  double x_max_jerk = 0.0;
  if (fabs(x_diff) > 1e-3) {
    // Constraints
    x_q_0_ = normalized_x_angle;

    double x_q_1 = position_x_;

    double x_v_0 = 0;

    double x_v_1 = 0;

    double x_a_0 = 0;

    double x_a_1 = 0;

    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 1> constraints;
    A << 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 5, 4, 3, 2, 1, 0, 0, 0, 0, 2, 0, 0,
        20, 12, 6, 2, 0, 0;

    constraints << x_q_0_, x_q_1, x_v_0, x_v_1, x_a_0, x_a_1;
    Eigen::Matrix<double, 6, 1> x_params = A.colPivHouseholderQr().solve(constraints);
    x_a_ = capValue(x_params(0));
    x_b_ = capValue(x_params(1));
    x_c_ = capValue(x_params(2));
    x_d_ = capValue(x_params(3));
    x_e_ = capValue(x_params(4));
    x_f_ = capValue(x_params(5));
    ROS_WARN_STREAM("x_params: " << x_params);

    std::vector<double> t_max;
    double x_t_max_1 = -x_b_ / (5.0 * x_a_) +
                       sqrt((4.0 * x_b_ * x_b_) / (100.0 * x_a_ * x_a_) - x_c_ / (10.0 * x_a_));
    if (x_t_max_1 >= 0.0 && x_t_max_1 <= 1.0) {
      t_max.emplace_back(x_t_max_1);
    }
    double x_t_max_2 = -x_b_ / (5.0 * x_a_) -
                       sqrt((4.0 * x_b_ * x_b_) / (100.0 * x_a_ * x_a_) - x_c_ / (10.0 * x_a_));
    if (x_t_max_2 >= 0.0 && x_t_max_2 <= 1.0) {
      t_max.emplace_back(x_t_max_2);
    }
    std::vector<double> x_max_accelerations;
    for (const auto& t : t_max) {
      double x_max_acceleration =
          20 * x_a_ * t * t * t + 12 * x_b_ * t * t + 6 * x_c_ * t + 2 * x_d_;
      // ROS_WARN_STREAM("x_max_acceleration: " << x_max_acceleration);
      x_max_accelerations.emplace_back(fabs(x_max_acceleration));
    }
    x_max_acceleration = *std::max_element(x_max_accelerations.begin(), x_max_accelerations.end());
    ROS_WARN_STREAM("x_max_acceleration: " << x_max_acceleration);

    double t_x_max_jerk = -24.0 * x_b_ / (120.0 * x_a_);
    x_max_jerk =
        60.0 * x_a_ * t_x_max_jerk * t_x_max_jerk + 24.0 * x_b_ * t_x_max_jerk + 6.0 * x_c_;
    ROS_WARN_STREAM("x max jerk: " << x_max_jerk);
  } else {
    x_a_ = 0.0;
    x_b_ = 0.0;
    x_c_ = 0.0;
    x_d_ = 0.0;
    x_e_ = 0.0;
    x_f_ = 0.0;
  }

  double y_max_acceleration = 0.0;
  double y_max_jerk = 0.0;
  if (fabs(y_diff) > 1e-3) {
    // Constraints
    y_q_0_ = normalized_y_angle;

    double y_q_1 = position_y_;

    double y_v_0 = 0;

    double y_v_1 = 0;

    double y_a_0 = 0;

    double y_a_1 = 0;

    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 1> constraints;

    A << 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 5, 4, 3, 2, 1, 0, 0, 0, 0, 2, 0, 0,
        20, 12, 6, 2, 0, 0;

    constraints << y_q_0_, y_q_1, y_v_0, y_v_1, y_a_0, y_a_1;
    Eigen::Matrix<double, 6, 1> y_params = A.colPivHouseholderQr().solve(constraints);
    y_a_ = capValue(y_params(0));
    y_b_ = capValue(y_params(1));
    y_c_ = capValue(y_params(2));
    y_d_ = capValue(y_params(3));
    y_e_ = capValue(y_params(4));
    y_f_ = capValue(y_params(5));
    ROS_WARN_STREAM("y_params: " << y_params);

    std::vector<double> t_max;
    double y_t_max_1 =
        -y_b_ / (5 * y_a_) + sqrt((4 * y_b_ * y_b_) / (100 * y_a_ * y_a_) - (y_c_) / (10 * y_a_));
    if (y_t_max_1 >= 0.0 && y_t_max_1 <= 1.0) {
      t_max.emplace_back(y_t_max_1);
    }
    double y_t_max_2 =
        -y_b_ / (5 * y_a_) - sqrt((4 * y_b_ * y_b_) / (100 * y_a_ * y_a_) - (y_c_) / (10 * y_a_));
    if (y_t_max_2 >= 0.0 && y_t_max_2 <= 1.0) {
      t_max.emplace_back(y_t_max_2);
    }
    std::vector<double> y_max_accelerations;
    for (const auto& t : t_max) {
      double y_max_acceleration =
          20 * y_a_ * t * t * t + 12 * y_b_ * t * t + 6 * y_c_ * t + 2 * y_d_;
      // ROS_WARN_STREAM("y_max_acceleration: " << y_max_acceleration);
      y_max_accelerations.emplace_back(fabs(y_max_acceleration));
    }
    y_max_acceleration = *std::max_element(y_max_accelerations.begin(), y_max_accelerations.end());
    ROS_WARN_STREAM("y_max_acceleration: " << y_max_acceleration);

    double t_y_max_jerk = -24 * y_b_ / (120 * y_a_);
    y_max_jerk =
        60.0 * y_a_ * t_y_max_jerk * t_y_max_jerk + 24.0 * y_b_ * t_y_max_jerk + 6.0 * y_c_;
    ROS_WARN_STREAM("y max jerk: " << y_max_jerk);
  } else {
    y_a_ = 0.0;
    y_b_ = 0.0;
    y_c_ = 0.0;
    y_d_ = 0.0;
    y_e_ = 0.0;
    y_f_ = 0.0;
  }

  // double max_velocity = 2.5 /*rad/s*/;
  // double max_acceleration = 25.0 /*rad/s*/;
  // t_end_ = 25.0 /*m/s^2*/ / std::max(x_max_acceleration, y_max_acceleration) * 1e-3;
  t_end_ = std::max(x_max_acceleration, y_max_acceleration);
  ROS_WARN_STREAM("t_end_: " << t_end_);
  t_end_ = std::min(t_end_, 0.3);
  ROS_WARN_STREAM("t_end_: " << t_end_);

  elapsed_time_ = ros::Duration(0.0);
}

void BalanceControllerCartesian::controllCallback(const std_msgs::Bool::ConstPtr& msg) {
  control_position_ = msg->data;
  ROS_WARN_STREAM("control_position_: " << control_position_);
}

void BalanceControllerCartesian::positionXCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (!position_initialized_) {
    position_x_ = msg->data * M_PI / 180;
    ROS_WARN_STREAM("position_x_: " << msg->data << "deg");

    T_base_end_current_.matrix() << current_pose_[0], current_pose_[4], current_pose_[8],
        current_pose_[12], current_pose_[1], current_pose_[5], current_pose_[9], current_pose_[13],
        current_pose_[2], current_pose_[6], current_pose_[10], current_pose_[14], current_pose_[3],
        current_pose_[7], current_pose_[11], current_pose_[15];
    Eigen::Vector3d euler_angles = T_base_end_current_.linear().eulerAngles(2, 1, 0);
    position_y_ = constrainAngle(euler_angles(1));

    x_values_.clear();
    // x_velocity_.clear();
    x_acceleration_.clear();
    x_acceleration_delta_.clear();
    y_values_.clear();
    // y_velocity_.clear();
    y_acceleration_.clear();
    y_acceleration_delta_.clear();
    goToPosition();
    position_initialized_ = true;
  }
}

void BalanceControllerCartesian::positionYCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (!position_initialized_) {
    position_y_ = msg->data * M_PI / 180;
    ROS_WARN_STREAM("position_y_: " << msg->data << "deg");

    T_base_end_current_.matrix() << current_pose_[0], current_pose_[4], current_pose_[8],
        current_pose_[12], current_pose_[1], current_pose_[5], current_pose_[9], current_pose_[13],
        current_pose_[2], current_pose_[6], current_pose_[10], current_pose_[14], current_pose_[3],
        current_pose_[7], current_pose_[11], current_pose_[15];
    Eigen::Vector3d euler_angles = T_base_end_current_.linear().eulerAngles(2, 1, 0);
    position_x_ = constrainAngle(euler_angles(2));

    x_values_.clear();
    // x_velocity_.clear();
    x_acceleration_.clear();
    x_acceleration_delta_.clear();
    y_values_.clear();
    // y_velocity_.clear();
    y_acceleration_.clear();
    y_acceleration_delta_.clear();
    goToPosition();
    position_initialized_ = true;
  }
}

}  // namespace balance_controller

PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceControllerCartesian,
                       controller_interface::ControllerBase)
