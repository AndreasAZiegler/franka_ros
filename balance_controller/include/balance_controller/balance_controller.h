// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <ball_tracker_msgs/TrackingUpdate.h>
#include <balance_controller/PlaneBoundaries.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <control_toolbox/pid.h>
#include <Eigen/Dense>

namespace balance_controller {

class BalanceController : public controller_interface::MultiInterfaceController<
                              hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  void publishTargetState();
  void publishCurrentState();
  void controllCallback(const std_msgs::Bool::ConstPtr& msg);
  void trackingCallback(const ball_tracker_msgs::TrackingUpdate::ConstPtr& msg);
  void boundariesCallback(const balance_controller::PlaneBoundaries::ConstPtr& msg);

  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 7> current_pose_{};

  Vector7d q_target_;  // Target positions of the arm [rad, rad, rad, rad, rad, rad, rad]
  std::mutex target_mutex_;

  realtime_tools::RealtimePublisher<sensor_msgs::JointState> target_position_publisher_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> current_position_publisher_;
  ros::Subscriber control_subscriber_;
  ros::Subscriber tracking_subscriber_;
  //ros::Subscriber boundaries_subscriber_;
  
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
  control_toolbox::Pid pid_x_;
  control_toolbox::Pid pid_y_;
  control_toolbox::Pid pid_x_init_;
  control_toolbox::Pid pid_y_init_;
  ros::Time last_time_;
};

}  // namespace balance_controller
