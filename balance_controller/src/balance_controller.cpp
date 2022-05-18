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

  publisher_.init(node_handle, "desired_position", 1);
  publisher_.lock();
  publisher_.msg_.name.resize(7);
  publisher_.msg_.position.resize(7);
  publisher_.msg_.velocity.resize(7);
  publisher_.msg_.effort.resize(7);
  publisher_.unlock();

  joint_position_subscriber_ =
      node_handle.subscribe("joint_position", 1000, &BalanceController::positionCallback, this);
  tracking_subscriber_ = node_handle.subscribe("/camera/tracking_update", 1000,
                                               &BalanceController::trackingCallback, this);
  // boundaries_subscriber_ = node_handle.subscribe("plane_boundaries", 10,
  // &BalanceController::boundariesCallback, this);

  position_initialized_ = false;

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

  /*
  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;
  for (size_t i = 0; i < 7; ++i) {
    if (i == 4) {
      position_joint_handles_[i].setCommand(current_pose_[i] - delta_angle);
    } else {
      position_joint_handles_[i].setCommand(current_pose_[i] + delta_angle);
    }
  }
  */
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      position_joint_handles_[i].setCommand(q_target_[i]);
    }
  }

  // position_joint_handles_[6].setCommand(q_target_[6]);

  publishTargetState();
}

void BalanceController::publishTargetState() {
  if (publisher_.trylock()) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      publisher_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      publisher_.msg_.position[i] = q_target_[i];
      //publisher_.msg_.velocity[i] = 0.0;
      //publisher_.msg_.effort[i] = leader_data_.tau_target[i];
    }
    publisher_.unlockAndPublish();
  }
}

void BalanceController::positionCallback(const std_msgs::Float64::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  q_target_[6] += msg->data;
}

void BalanceController::trackingCallback(const ball_tracker_msgs::TrackingUpdate::ConstPtr& msg) {
  position_initialized_ = true;
  x_current_ = msg->x;
  y_current_ = msg->y;

  double delta_angle = 0.0005;

  /*
  for (std::size_t i = 0; i < 7; ++i) {
    q_target_[i] = current_pose_[i];
  }
  */

  if ((x_current_ > x_middle_) && (x_current_ < x_max_)) {
    // increase
    // q_target_[6] = current_pose_[6] + delta_angle;
    delta_angle = std::abs(x_current_ - x_middle_) / ((x_max_ - x_min_) / 2.0) * delta_angle;
    {
      //ROS_INFO_STREAM("q_target_[6] before: " << q_target_[6]);
      //if (delta_angle > 0.25)
      if (q_target_[6] + delta_angle > 0.25)
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        q_target_[6] = 0.25;
      }
      else
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        q_target_[6] = q_target_[6] + delta_angle;
        //q_target_[6] = delta_angle;
      }
      //ROS_INFO_STREAM("q_target_[6] after: " << q_target_[6]);
    }
    //ROS_INFO_STREAM("increased");
  } else if ((x_current_ < x_middle_) && (x_current_ > x_min_)) {
    // decrease
    // q_target_[6] = current_pose_[6] - delta_angle;
    delta_angle = std::abs(x_current_ - x_middle_) / ((x_max_ - x_min_) / 2.0) * delta_angle;
    {
      //ROS_INFO_STREAM("q_target_[6] before: " << q_target_[6]);
      //if (delta_angle < -0.25)
      if (q_target_[6] - delta_angle < -0.25)
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        q_target_[6] = -0.25;
      }
      else
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        q_target_[6] = q_target_[6] - delta_angle;
        //q_target_[6] = delta_angle;
      }
      //ROS_INFO_STREAM("q_target_[6] after: " << q_target_[6]);
    }
    //ROS_INFO_STREAM("decreased");
  }
}

/*
void BalanceController::boundariesCallback(const balance_controller::PlaneBoundaries::ConstPtr& msg)
{
}
*/

}  // namespace balance_controller

PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceController, controller_interface::ControllerBase)
