// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <balance_controller/balance_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace balance_controller {

bool BalanceController::init(hardware_interface::RobotHW* robot_hardware,
                             ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR("BalanceController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("BalanceController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("BalanceController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("BalanceController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  if (!node_handle.getParam("x_min", x_min_)) {
    ROS_ERROR("BalanceController: Could not parse x_min");
  }
  if (!node_handle.getParam("x_max", x_max_)) {
    ROS_ERROR("BalanceController: Could not parse x_max");
  }
  if (!node_handle.getParam("y_min", y_min_)) {
    ROS_ERROR("BalanceController: Could not parse y_min");
  }
  if (!node_handle.getParam("y_max", y_max_)) {
    ROS_ERROR("BalanceController: Could not parse y_max");
  }

  x_middle_ = (x_max_ + x_min_) / 2;
  y_middle_ = (y_max_ + y_min_) / 2;

  /*
  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "BalanceController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch balance_controller move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }
  */

  target_position_publisher_.init(node_handle, "desired_position", 1);
  target_position_publisher_.lock();
  target_position_publisher_.msg_.name.resize(7);
  target_position_publisher_.msg_.position.resize(7);
  target_position_publisher_.msg_.velocity.resize(7);
  target_position_publisher_.msg_.effort.resize(7);
  target_position_publisher_.unlock();

  current_position_publisher_.init(node_handle, "current_position", 1);
  current_position_publisher_.lock();
  current_position_publisher_.msg_.name.resize(7);
  current_position_publisher_.msg_.position.resize(7);
  current_position_publisher_.msg_.velocity.resize(7);
  current_position_publisher_.msg_.effort.resize(7);
  current_position_publisher_.unlock();

  control_subscriber_ =
      node_handle.subscribe("control", 10, &BalanceController::controllCallback, this);
  tracking_subscriber_ = node_handle.subscribe("/camera/tracking_update", 1000,
                                               &BalanceController::trackingCallback, this);
  // boundaries_subscriber_ = node_handle.subscribe("plane_boundaries", 10,
  // &BalanceController::boundariesCallback, this);

  position_initialized_ = false;

  control_position_ = false;
  //	initPid (double p, double i, double d, double i_max, double i_min, bool antiwindup=false)
  pid_x_.initPid(0.0003/*p*/, 0.00005/*i*/, 0.0/*d*/, 0.3/*i_max*/, -0.3/*i_min*/, true/*antiwindup*/);
  pid_y_.initPid(0.0003/*p*/, 0.0/*i*/, 0.0/*d*/, 0.3/*i_max*/, -0.3/*i_min*/, true/*antiwindup*/);
  pid_x_init_.initPid(0.35/*p*/, 0.0/*i*/, 0.0/*d*/, 0.3/*i_max*/, -0.3/*i_min*/, true/*antiwindup*/);
  pid_y_init_.initPid(0.35/*p*/, 0.0/*i*/, 0.0/*d*/, 0.5/*i_max*/, -0.5/*i_min*/, true/*antiwindup*/);
  last_time_ = ros::Time::now();

  return true;
}

void BalanceController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    current_pose_[i] = position_joint_handles_[i].getPosition();
  }
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (std::size_t i = 0; i < 7; ++i) {
      q_target_[i] = current_pose_[i];
    }
  }

  elapsed_time_ = ros::Duration(0.0);
}

void BalanceController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  elapsed_time_ += period;


  for (std::size_t i = 0; i < 7; ++i) {
    q_target_[i] = current_pose_[i];
  }

  if (control_position_)
  {
    int x_current;
    int y_current;
    {
      std::lock_guard<std::mutex> lock(current_mutex_);
      x_current = x_current_;
      y_current = y_current_;
    }

    if ((x_current < x_max_) && (x_current > x_min_))
    {
      ros::Time time = ros::Time::now();
      double effort_x = pid_x_.computeCommand(x_current - x_middle_, time - last_time_);
      double y_diff = y_current - y_middle_;
      double effort_y = 0.0;
      if (y_diff > 0.1)
      {
        effort_y = pid_y_.computeCommand(y_diff, time - last_time_);
      }
      last_time_ = time;

      std::lock_guard<std::mutex> lock(target_mutex_);
      q_target_[5] = q_target_[5] + effort_y;
      q_target_[6] = q_target_[6] + effort_x;
    }
  }
  else
  {
    ros::Time time = ros::Time::now();
    double effort_x = pid_x_init_.computeCommand(current_pose_[6] - 0.04, time - last_time_);
    double effort_y = pid_y_init_.computeCommand(current_pose_[5] - 2.95, time - last_time_);
    last_time_ = time;
    q_target_[5] = q_target_[5] - effort_y;
    q_target_[6] = q_target_[6] - effort_x;
  }

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      position_joint_handles_[i].setCommand(q_target_[i]);
    }
  }

  publishTargetState();
  publishCurrentState();
}

void BalanceController::publishTargetState() {
  if (target_position_publisher_.trylock()) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      target_position_publisher_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      target_position_publisher_.msg_.position[i] = q_target_[i];
      // publisher_.msg_.velocity[i] = 0.0;
      // publisher_.msg_.effort[i] = leader_data_.tau_target[i];
    }
    target_position_publisher_.unlockAndPublish();
  }
}

void BalanceController::publishCurrentState() {
  if (current_position_publisher_.trylock()) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      current_position_publisher_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      current_position_publisher_.msg_.position[i] = current_pose_[i];
      // publisher_.msg_.velocity[i] = 0.0;
      // publisher_.msg_.effort[i] = leader_data_.tau_target[i];
    }
    current_position_publisher_.unlockAndPublish();
  }
}


void BalanceController::controllCallback(const std_msgs::Bool::ConstPtr& msg) {
  control_position_ = msg->data; 
  ROS_WARN_STREAM("control_position_: " << control_position_);
}

void BalanceController::trackingCallback(const ball_tracker_msgs::TrackingUpdate::ConstPtr& msg) {
  position_initialized_ = true;
  {
    std::lock_guard<std::mutex> lock(current_mutex_);
    x_current_ = msg->x;
    y_current_ = msg->y;
  }

  double delta_angle = 0.001;
}

/*
void BalanceController::boundariesCallback(const balance_controller::PlaneBoundaries::ConstPtr& msg)
{
}
*/

}  // namespace balance_controller

PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceController, controller_interface::ControllerBase)
