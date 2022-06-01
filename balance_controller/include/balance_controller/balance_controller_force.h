// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <ball_tracker_msgs/TrackingUpdate.h>

//#include <franka_example_controllers/desired_mass_paramConfig.h>

namespace balance_controller {

class BalanceControllerForce : public controller_interface::MultiInterfaceController<
                                   franka_hw::FrankaModelInterface,
                                   hardware_interface::EffortJointInterface,
                                   franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  // Saturation
  //Eigen::Matrix<double, 7, 1> saturateTorqueRate(
  //    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
  //    const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  //double desired_mass_{0.0};
  //double target_mass_{0.0};
  //double k_p_{0.0};
  //double k_i_{0.0};
  //double target_k_p_{0.0};
  //double target_k_i_{0.0};
  //double filter_gain_{0.001};
  //Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  //Eigen::Matrix<double, 7, 1> tau_error_;
  //static constexpr double kDeltaTauMax{1.0};

  std::array<double, 7> initial_pose_{};
  std::array<double, 7> current_pose_{};
  std::array<double, 7> current_error_{};
  std::array<double, 7> desired_position_ {};
  Vector7d tau_target_;  // Target positions of the arm [rad, rad, rad, rad, rad, rad, rad]

  std::mutex target_mutex_;

  void publishTargetState();
  void publishCurrentState();
  void controllCallback(const std_msgs::Bool::ConstPtr& msg);
  void angularPositionXCallback(const std_msgs::Float32::ConstPtr& msg);
  void angularPositionYCallback(const std_msgs::Float32::ConstPtr& msg);
  void trackingCallback(const ball_tracker_msgs::TrackingUpdate::ConstPtr& msg);

  realtime_tools::RealtimePublisher<sensor_msgs::JointState> target_position_publisher_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> current_position_publisher_;
  ros::Subscriber control_subscriber_;
  ros::Subscriber angular_position_x_subscriber_;
  ros::Subscriber angular_position_y_subscriber_;
  ros::Subscriber tracking_subscriber_;

  // Dynamic reconfigure
  /*
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::desired_mass_paramConfig>>
      dynamic_server_desired_mass_param_;
  ros::NodeHandle dynamic_reconfigure_desired_mass_param_node_;
  void desiredMassParamCallback(franka_example_controllers::desired_mass_paramConfig& config,
                                uint32_t level);
  */
  int x_min_;
  int x_max_;
  int y_min_;
  int y_max_;
  int x_middle_;
  int y_middle_;

  bool position_initialized_;
  int x_current_;
  int y_current_;
  std::mutex current_mutex_;

  bool control_position_;
  double angular_position_x_;
  double angular_position_y_;
  std::array<control_toolbox::Pid, 5> const_joint_pid_;
  control_toolbox::Pid pid_x_position_;
  control_toolbox::Pid pid_y_position_;
  control_toolbox::Pid pid_x_angular_position_;
  control_toolbox::Pid pid_y_angular_position_;
  ros::Time last_time_;
};

}  // namespace franka_example_controllers
