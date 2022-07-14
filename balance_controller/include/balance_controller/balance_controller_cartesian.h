// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <geometry_msgs/Transform.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

namespace balance_controller {

class BalanceControllerCartesian
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  void goToPosition();

  void publishTargetState(const Eigen::Transform<double, 3, Eigen::Affine>& target_pose);
  void publishCurrentState();

  void controllCallback(const std_msgs::Bool::ConstPtr& msg);
  ros::Subscriber control_subscriber_;

  void positionXCallback(const std_msgs::Float32::ConstPtr& msg);
  ros::Subscriber position_x_subscriber_;
  void positionYCallback(const std_msgs::Float32::ConstPtr& msg);
  ros::Subscriber position_y_subscriber_;

  realtime_tools::RealtimePublisher<geometry_msgs::Transform> target_pose_publisher_;
  std::mutex target_mutex_;
  realtime_tools::RealtimePublisher<geometry_msgs::Transform> current_pose_publisher_;
  std::mutex current_mutex_;

  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

  bool control_position_;
  bool position_initialized_;

  std::array<double, 16> current_pose_;
  double position_x_;
  double position_y_;

  ros::Duration elapsed_time_;

  Eigen::Transform<double, 3, Eigen::Affine> T_base_end_current_;
  Eigen::Transform<double, 3, Eigen::Affine> T_base_end_ref_;
  Eigen::Transform<double, 3, Eigen::Affine> T_offset_;
  std::vector<Eigen::Matrix4d> waypoints_;
  std::vector<double> periods_;

  std::vector<double> x_values_;
  std::vector<double> y_values_;
  std::vector<double> measured_joint_velocity_;
  std::vector<double> x_velocity_;
  std::vector<double> y_velocity_;
  std::vector<double> x_velocity_deviation_;
  std::vector<double> y_velocity_deviation_;
  std::vector<double> x_velocity_delta_;
  std::vector<double> y_velocity_delta_;
  std::vector<double> x_acceleration_;
  std::vector<double> y_acceleration_;
  std::vector<double> x_acceleration_deviation_;
  std::vector<double> y_acceleration_deviation_;
  std::vector<double> x_acceleration_delta_;
  std::vector<double> y_acceleration_delta_;
  double previous_x_value_;
  double previous_y_value_;
  double previous_x_velocity_;
  double previous_y_velocity_;
  double previous_x_acceleration_;
  double previous_y_acceleration_;

  double x_q_0_;
  double y_q_0_;
  double x_a_;
  double x_b_;
  double x_c_;
  double x_d_;
  double x_e_;
  double x_f_;
  double y_a_;
  double y_b_;
  double y_c_;
  double y_d_;
  double y_e_;
  double y_f_;
  double t_end_;
};

}  // namespace balance_controller
