// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <balance_controller/balance_controller_force.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace balance_controller {

bool BalanceControllerForce::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "BalanceControllerForce: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("BalanceControllerForce: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "BalanceControllerForce: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("BalanceControllerForce: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "BalanceControllerForce: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("BalanceControllerForce: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "BalanceControllerForce: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("BalanceControllerForce: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("BalanceControllerForce: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  /*
  dynamic_reconfigure_desired_mass_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_desired_mass_param_node");
  dynamic_server_desired_mass_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::desired_mass_paramConfig>>(

      dynamic_reconfigure_desired_mass_param_node_);
  dynamic_server_desired_mass_param_->setCallback(
      boost::bind(&BalanceControllerForce::desiredMassParamCallback, this, _1, _2));
  */

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
  ROS_WARN_STREAM("x_middle_: " << x_middle_);
  ROS_WARN_STREAM("y_middle_: " << y_middle_);

  double joint_1_steady_position;
  if (!node_handle.getParam("joint_1_steady_position", joint_1_steady_position)) {
    ROS_ERROR("BalanceController: Could not parse joint_1_steady_position");
  }
  initial_pose_[0] = joint_1_steady_position;
  double joint_2_steady_position;
  if (!node_handle.getParam("joint_2_steady_position", joint_2_steady_position)) {
    ROS_ERROR("BalanceController: Could not parse joint_2_steady_position");
  }
  initial_pose_[1] = joint_2_steady_position;
  double joint_3_steady_position;
  if (!node_handle.getParam("joint_3_steady_position", joint_3_steady_position)) {
    ROS_ERROR("BalanceController: Could not parse joint_3_steady_position");
  }
  initial_pose_[2] = joint_3_steady_position;
  double joint_4_steady_position;
  if (!node_handle.getParam("joint_4_steady_position", joint_4_steady_position)) {
    ROS_ERROR("BalanceController: Could not parse joint_4_steady_position");
  }
  initial_pose_[3] = joint_4_steady_position;
  double joint_5_steady_position;
  if (!node_handle.getParam("joint_5_steady_position", joint_5_steady_position)) {
    ROS_ERROR("BalanceController: Could not parse joint_5_steady_position");
  }
  initial_pose_[4] = joint_5_steady_position;
  double joint_6_steady_position;
  if (!node_handle.getParam("joint_6_steady_position", joint_6_steady_position)) {
    ROS_ERROR("BalanceController: Could not parse joint_6_steady_position");
  }
  initial_pose_[5] = joint_6_steady_position;
  ROS_INFO_STREAM("initial_pose_[5]: " << initial_pose_[5]);
  double joint_7_steady_position;
  if (!node_handle.getParam("joint_7_steady_position", joint_7_steady_position)) {
    ROS_ERROR("BalanceController: Could not parse joint_7_steady_position");
  }
  initial_pose_[6] = joint_7_steady_position;
  ROS_INFO_STREAM("initial_pose_[6]: " << initial_pose_[6]);

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

  calibrate_subscriber_ =
      node_handle.subscribe("calibrate_pid", 10, &BalanceControllerForce::calibrateCallback, this);
  control_subscriber_ =
      node_handle.subscribe("control", 10, &BalanceControllerForce::controllCallback, this);
  angular_position_x_subscriber_ = node_handle.subscribe(
      "angular_position_x", 10, &BalanceControllerForce::angularPositionXCallback, this);
  angular_position_y_subscriber_ = node_handle.subscribe(
      "angular_position_y", 10, &BalanceControllerForce::angularPositionYCallback, this);
  tracking_subscriber_ = node_handle.subscribe("/camera/tracking_update", 1000,
                                               &BalanceControllerForce::trackingCallback, this);
  // boundaries_subscriber_ = node_handle.subscribe("plane_boundaries", 10,
  // &BalanceController::boundariesCallback, this);

  position_initialized_ = false;

  calibrate_pid_ = false;
  control_position_ = false;
  //	initPid (double p, double i, double d, double i_max, double i_min, bool antiwindup=false)
  for (auto& pid : const_joint_pid_) {
    pid.initPid(30.0 /*p*/, 5.0 /*i*/, 0.0 /*d*/, 0.8 /*i_max*/, -0.8 /*i_min*/,
                false /*antiwindup*/);
  }
  // const_joint_pid_.at(2).initPid(25.0/*p*/, 0.01/*i*/, 0.0/*d*/, 0.5/*i_max*/, -0.5/*i_min*/,
  // true/*antiwindup*/);

  pid_x_joint_position_.initPid(40.0 /*p*/, 0.0 /*i*/, 1.5 /*d*/, 0.8 /*i_max*/, -0.8 /*i_min*/,
                                true /*antiwindup*/);
  pid_y_joint_position_.initPid(10.0 /*p*/, 0.0 /*i*/, 0.0 /*d*/, 0.8 /*i_max*/, -0.8 /*i_min*/,
                                true /*antiwindup*/);

  pid_x_position_.initPid(0.000225 /*p*/, 0.0 /*i*/, 0.0 /*d*/, 0.0 /*i_max*/, -0.2 /*i_min*/,
                          true /*antiwindup*/);
  pid_y_position_.initPid(0.00015 /*p*/, 0.0 /*i*/, 0.0 /*d*/, 0.0 /*i_max*/, -0.15 /*i_min*/,
                          true /*antiwindup*/);
  last_time_ = ros::Time::now();

  return true;
}

void BalanceControllerForce::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  // tau_ext_initial_ = tau_measured - gravity;
  // tau_error_.setZero();

  for (size_t i = 0; i < 7; ++i) {
    current_pose_[i] = joint_handles_[i].getPosition();
  }

  angular_position_x_ = initial_pose_[5];
  angular_position_y_ = initial_pose_[6];
}

void BalanceControllerForce::update(const ros::Time& time, const ros::Duration& period) {
  for (size_t i = 0; i < 7; ++i) {
    current_pose_[i] = joint_handles_[i].getPosition();
    current_error_[i] = 0.0;
    desired_position_[i] = 0.0;
  }

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (std::size_t i = 0; i < 7; ++i) {
      tau_target_[i] = 0.0;
    }
  }

  auto angular_position_x = angular_position_x_;
  auto angular_position_y = angular_position_y_;
  // ros::Time time = ros::Time::now();
  if (control_position_ && position_initialized_) {
    int x_current;
    int y_current;
    {
      std::lock_guard<std::mutex> lock(current_mutex_);
      x_current = x_current_;
      y_current = y_current_;
    }

    if ((x_current < x_max_) && (x_current > x_min_)) {
      double joint_position_x =
          pid_x_position_.computeCommand(x_middle_ - x_current, time - last_time_);
      angular_position_x = -joint_position_x + initial_pose_[6];
      double joint_position_y =
          pid_y_position_.computeCommand(y_middle_ - y_current, time - last_time_);
      angular_position_y = -joint_position_y + initial_pose_[5];

      desired_position_[6] = angular_position_x;
      desired_position_[5] = angular_position_y;

      //current_error_[6] = angular_position_x - current_pose_[6];
      //current_error_[5] = angular_position_y - current_pose_[5];
      current_error_[5] = x_middle_ - x_current;
      current_error_[6] = angular_position_x - current_pose_[6];
      double effort_x = pid_x_joint_position_.computeCommand(angular_position_x - current_pose_[6],
                                                             time - last_time_);
      double effort_y = pid_y_joint_position_.computeCommand(angular_position_y - current_pose_[5],
                                                             time - last_time_);

      /*
      if (-effort_x < 0)
      {
        effort_x *= 1.4;
      }
      if (effort_y < 0)
      {
        effort_y *= 1.2;
      }
      */

      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        // tau_target_[5] = -effort_y;
        tau_target_[6] = effort_x;
      }
    }
  } else {
    desired_position_[6] = initial_pose_[6];
    desired_position_[5] = initial_pose_[5];

    current_error_[6] = initial_pose_[6] - current_pose_[6];
    current_error_[5] = initial_pose_[5] - current_pose_[5];

    if (calibrate_pid_) {
      std::lock_guard<std::mutex> lock(target_mutex_);
      tau_target_[6] = pid_x_joint_position_.computeCommand(initial_pose_[6] - current_pose_[6],
                                                            time - last_time_);
      tau_target_[5] = pid_y_joint_position_.computeCommand(initial_pose_[5] - current_pose_[5],
                                                            time - last_time_);
    } else {
      std::lock_guard<std::mutex> lock(target_mutex_);
      for (std::size_t i = 5; i < 7; ++i) {
        tau_target_[i] = const_joint_pid_[i].computeCommand(initial_pose_[i] - current_pose_[i],
                                                            time - last_time_);
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (std::size_t i = 0; i < 5; ++i) {
      tau_target_[i] = const_joint_pid_[i].computeCommand(initial_pose_[i] - current_pose_[i],
                                                          time - last_time_);
    }
  }
  last_time_ = time;

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_target_[i]);
    }
  }

  publishTargetState();
  publishCurrentState();
}

/*
void BalanceControllerForce::desiredMassParamCallback(
    franka_example_controllers::desired_mass_paramConfig& config,
    uint32_t ) {
  target_mass_ = config.desired_mass;
  target_k_p_ = config.k_p;
  target_k_i_ = config.k_i;
}
*/

/*
Eigen::Matrix<double, 7, 1> BalanceControllerForce::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}
*/

void BalanceControllerForce::publishTargetState() {
  if (target_position_publisher_.trylock()) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      target_position_publisher_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      target_position_publisher_.msg_.position[i] = desired_position_[i];
      // target_position_publisher_.msg_.position[i] = q_target_[i];
      // target_position_publisher_.msg_.velocity[i] = 0.0;
      target_position_publisher_.msg_.effort[i] = tau_target_[i];
    }
    target_position_publisher_.unlockAndPublish();
  }
}

void BalanceControllerForce::publishCurrentState() {
  if (current_position_publisher_.trylock()) {
    std::lock_guard<std::mutex> lock(target_mutex_);
    for (size_t i = 0; i < 7; ++i) {
      current_position_publisher_.msg_.name[i] = "panda_joint" + std::to_string(i + 1);
      current_position_publisher_.msg_.position[i] = current_pose_[i];
      current_position_publisher_.msg_.velocity[i] = current_error_[i];
      // publisher_.msg_.velocity[i] = 0.0;
      // publisher_.msg_.effort[i] = leader_data_.tau_target[i];
    }
    current_position_publisher_.unlockAndPublish();
  }
}

void BalanceControllerForce::calibrateCallback(const std_msgs::Bool::ConstPtr& msg) {
  calibrate_pid_ = msg->data;
  ROS_WARN_STREAM("calibrate_pid_: " << calibrate_pid_);
}

void BalanceControllerForce::controllCallback(const std_msgs::Bool::ConstPtr& msg) {
  control_position_ = msg->data;
  ROS_WARN_STREAM("control_position_: " << control_position_);
}

void BalanceControllerForce::angularPositionXCallback(const std_msgs::Float32::ConstPtr& msg) {
  angular_position_x_ = msg->data;
  initial_pose_[6] = angular_position_x_;
  ROS_WARN_STREAM("angular_position_x_: " << angular_position_x_);
}

void BalanceControllerForce::angularPositionYCallback(const std_msgs::Float32::ConstPtr& msg) {
  angular_position_y_ = msg->data;
  initial_pose_[5] = angular_position_y_;
  ROS_WARN_STREAM("angular_position_y_: " << angular_position_y_);
}

void BalanceControllerForce::trackingCallback(
    const ball_tracker_msgs::TrackingUpdate::ConstPtr& msg) {
  position_initialized_ = true;
  {
    std::lock_guard<std::mutex> lock(current_mutex_);
    x_current_ = msg->x;
    y_current_ = msg->y;
  }
}

}  // namespace balance_controller

PLUGINLIB_EXPORT_CLASS(balance_controller::BalanceControllerForce,
                       controller_interface::ControllerBase)
