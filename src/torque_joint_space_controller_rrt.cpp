#include <advanced_robotics_franka_controllers/torque_joint_space_controller.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/model.h>

#include "math_type_define.h"

#include "advanced_robotics_franka_controllers/torque_joint_space_controller_rrt.h"


namespace advanced_robotics_franka_controllers
{

bool TorqueJointSpaceControllerRRT::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  
  //ros::init( argc, argv, "assembly_vrep");
  joint0_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint0_data.txt","w");  
  gripper_ac_.waitForServer();
  gripper_ac_open.waitForServer();

  //joint_state_pub_ = node_handle.advertise<sensor_msgs::JointState>("/panda/left_joint_states", 1);
  // goal_state_pub_ = node_handle.advertise<geometry_msgs::Transform>("/panda/left_goal_trans", 1);

  planned_trajectory = node_handle.subscribe("/planned_arm_trajectory", 100, &TorqueJointSpaceControllerRRT::traj_cb, this);
  planned_done_ = node_handle.subscribe("/rrt_planned_done", 100, &TorqueJointSpaceControllerRRT::planned_cb, this);
  planned_done = false;

  gripper_sub_ = node_handle.subscribe("/gripper", 100, &TorqueJointSpaceControllerRRT::grip_cb, this);

  gripper_open_sub_ = node_handle.subscribe("/gripper_open", 100, &TorqueJointSpaceControllerRRT::grip_open_cb, this);

  task_start_sub_ = node_handle.subscribe("/task_start", 100, &TorqueJointSpaceControllerRRT::taskStartCallback,this);
  f_star_zero_sub_ = node_handle.subscribe("/f_star_zero_cmd", 100, &TorqueJointSpaceControllerRRT::commandForceCallback,this);

  gripper_done = false;
  gripper_open = false;
	std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void TorqueJointSpaceControllerRRT::traj_cb(const sensor_msgs::JointStateConstPtr& msg)
{
    for (size_t i = 0; i < msg->position.size(); i++)
    {
        q_traj_[i] = msg->position[i];
    }
}




void TorqueJointSpaceControllerRRT::planned_cb(const std_msgs::Bool& msg)
{
    planned_done = msg.data;
}

void TorqueJointSpaceControllerRRT::grip_cb(const std_msgs::Bool& msg)
{
    gripper_done = msg.data;
    gripper_open = false;
}

void TorqueJointSpaceControllerRRT::grip_open_cb(const std_msgs::Bool& msg)
{

    gripper_open = msg.data;
    gripper_done = false;
}

void TorqueJointSpaceControllerRRT::starting(const ros::Time& time) {
  start_time_ = time;

  for (size_t i = 0; i < 7; ++i)
  {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  //q_traj_ = q_init_;

  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  pos_init_ = transform_init_.translation();
  ori_init_ = transform_init_.rotation();

//////////////////////////////////////////////////////////////////



  current_velocity_info_.data.resize(6);
  for(size_t i = 0; i < 6; i++)
  {
    current_velocity_info_.data[i] = 0;
  }

  current_position_info_.x = 0;
  current_position_info_.y = 0;
  current_position_info_.z = 0;

  current_force_info_.force.x = 0;
  current_force_info_.force.y = 0;
  current_force_info_.force.z = 0;
  current_torque_info_.torque.x = 0;
  current_torque_info_.torque.y = 0;
  current_torque_info_.torque.z = 0;

  current_orientation_info_.data.resize(3);
  for (size_t i = 0; i < 3; i++)
  {
    current_orientation_info_.data[i] = 0;
  }

  f_star_zero_.setZero();
  peg_in_hole_state_ = 0;

  current_velocity_.setZero();
  current_position_.setZero();
  current_force_.setZero();
  current_torque_.setZero();

  is_first_ = true;
  the_first_tick_ = true;
  task_start_ = false;

}

void TorqueJointSpaceControllerRRT::update(const ros::Time& time, const ros::Duration& period) {

  const franka::RobotState &robot_state = state_handle_->getRobotState();
  const std::array<double, 42> &jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  const std::array<double, 7> &gravity_array = model_handle_->getGravity();
  const std::array<double, 49> &massmatrix_array = model_handle_->getMass();
  const std::array<double, 7> &coriolis_array = model_handle_->getCoriolis();

  const std::array<double, 3ul> gravity_dir = {{0., .0, 9.81}};
  double theta = 15.0;

  std::array<double, 7> gravity2 = model_handle_->getGravity({{9.81*sin(theta*M_PI/180.0), 0.0, -9.81*cos(theta*M_PI/180.0)}});//franka::Model::gravity(robot_state, gravity_dir);

  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(massmatrix_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> qd(robot_state.dq.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> graivity_mod(gravity2.data());


  Eigen::Matrix<double , 6, 7> jacobian_euler;
  Eigen::Matrix<double , 12, 7> jacobian_dc;
  Eigen::Matrix<double , 7, 1> q_goal;
  Eigen::Matrix<double , 7, 1> q_desired;
  Eigen::Matrix<double , 7, 1> qd_desired;
  Eigen::Matrix<double , 12, 1> x_goal; 
  Eigen::Matrix<double , 12, 1> x_desired;
  Eigen::Matrix<double , 12, 1> x_current;
  
  q_goal.setZero();
  q_goal << M_PI/2, 0.0, 0.0, -M_PI/2, 0, M_PI/2, M_PI/4;
  q_desired.setZero();

  //q_goal = q_init_;
  //q_goal(5) -= M_PI/6.0;

  ros::Duration simulation_time = time - start_time_;
  Eigen::Matrix<double, 7, 1> tau_cmd;
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M;//(transform.rotation());

	rotation_M = Rotate_with_Z(q(0))*Rotate_with_Y(q(1))*Rotate_with_Z(q(2))*Rotate_with_Y(-q(3))*Rotate_with_Z(q(4))*Rotate_with_Y(-q(5))*Rotate_with_Z(-(q(6)));

//////////////////////////////////////////////////////////////////////////
  Eigen::Vector6d x_dot_(jacobian*qd);

 f_sensing = (jacobian * jacobian.transpose()).inverse() * jacobian * (tau_measured - gravity);
  
  current_velocity_ = x_dot_;
  current_position_ = position;
  current_orientation_ = DyrosMath::rot2Euler(rotation_M);
  current_force_ = f_sensing.head<3>();
  current_torque_ = f_sensing.tail<3>();

  for(size_t i = 0; i < 6; i++)
  {
    current_velocity_info_.data[i] = current_velocity_[i];
  }

  current_position_info_.x = current_position_(0);
  current_position_info_.y = current_position_(1);
  current_position_info_.z = current_position_(2);

  for(size_t i = 0; i < 3; i++)
  {
    current_orientation_info_.data[i] = current_orientation_[i];
  }

  current_force_info_.force.x = current_force_(0);
  current_force_info_.force.y = current_force_(1);
  current_force_info_.force.z = current_force_(2);
  current_torque_info_.torque.x = current_torque_(0);
  current_torque_info_.torque.y = current_torque_(1);
  current_torque_info_.torque.z = current_torque_(2);

  // current_velocity_pub_.publish(current_velocity_info_);
  // current_position_pub_.publish(current_position_info_);
  // current_orientation_pub_.publish(current_orientation_info_);
  // current_force_pub_.publish(current_force_info_);

  //////////////////////////////////////////////////////////////////////////

  double trajectory_time = 5.0;
  for(int i=0; i<7;i++)
  {
    q_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
                                        q_init_(i), q_goal(i), 0, 0);
    qd_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
                                        q_init_(i), q_goal(i), 0, 0);

  }
  // goal_trans_state_.translation
  // goal_trans_state_.rotation
  qd_desired.setZero();


  double kp, kv;
  kp = 2000;
  kv = 10;

  Eigen::Matrix<double, 7, 7> Kp_;
  Eigen::Matrix<double, 7, 7> Kd_;

  Kp_.setIdentity();
  Kd_.setIdentity();

  for (int i = 0; i < 7; i++)
  {
    Kp_(i, i) = 800.0;
    Kd_(i, i) = 10.0;
  }
  Kp_(6, 6) = 300.0;
  Kd_(6, 6) = 5.0;
  if (!planned_done)
  {
    q_traj_ = q;
  }
  tau_cmd = (Kp_ * (q_traj_ - q) - Kd_ * qd); // + coriolis;

  if (task_start_)
  {
    tau_cmd = jacobian.transpose() * (f_star_zero_);
  }
  // tau_cmd = -gravity + graivity_mod;
  // tau_cmd.setZero();
  //tau_cmd = (Kp_ * (q_traj_ - q) - Kd_ * qd); // + coriolis;

  //tau_cmd.setZero();
  //std::cout << q_traj_.transpose() <<std::endl;
  if (print_rate_trigger_()) {
    
    // ROS_INFO("--------------------------------------------------");
    // ROS_INFO_STREAM("tau :" << gravity.transpose());
    // //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    // //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    // ROS_INFO_STREAM("time :"<< simulation_time);
    // ROS_INFO_STREAM("q_curent : "<< graivity_mod.transpose());
    // ROS_INFO_STREAM("q_desired : "<< q_desired.transpose());
    // ROS_INFO_STREAM("q_traj_ : " << q.transpose());
  //  ROS_INFO_STREAM("current pos : " << position.transpose());
    //ROS_INFO_STREAM("f_star_zero_ : " <<f_star_zero_.transpose());
    ROS_INFO_STREAM("rotation_M : " <<rotation_M);

  }

  fprintf(joint0_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", time.toSec(), q_desired(0), q(0), qd(0), tau_cmd(0), tau_J_d(0), tau_measured(0), mass_matrix(0, 0));
  //fprintf(joint0_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_star_zero_(0), f_star_zero_(1), f_star_zero_(2), f_star_zero_(0), f_star_zero_(1), f_star_zero_(2));
  
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

    if(gripper_done){
      franka_gripper::GraspGoal goal;
      franka_gripper::GraspEpsilon epsilon;
      epsilon.inner = 0.01;
      epsilon.outer = 0.01;
      goal.speed = 0.1;
      goal.width = 0.05;
      goal.force = 100.0;
      goal.epsilon = epsilon;
      gripper_ac_.sendGoal(goal);
    
    }

    if(gripper_open){
      franka_gripper::MoveGoal goal;
      goal.speed = 0.1;
      goal.width = 0.08;
    
      gripper_ac_open.sendGoal(goal);
      }
}


Matrix3d TorqueJointSpaceControllerRRT::Rotate_with_X(const double rAngle)
{
	Matrix3d _Rotate_wth_X;

	_Rotate_wth_X(0, 0) = 1.0;
	_Rotate_wth_X(1, 0) = 0.0;
	_Rotate_wth_X(2, 0) = 0.0;

	_Rotate_wth_X(0, 1) = 0.0;
	_Rotate_wth_X(1, 1) = cos(rAngle);
	_Rotate_wth_X(2, 1) = sin(rAngle);

	_Rotate_wth_X(0, 2) = 0.0;
	_Rotate_wth_X(1, 2) = -sin(rAngle);
	_Rotate_wth_X(2, 2) = cos(rAngle);

	return(_Rotate_wth_X);
}

Matrix3d TorqueJointSpaceControllerRRT::Rotate_with_Y(const double rAngle)
{
	Matrix3d _Rotate_wth_Y(3, 3);

	_Rotate_wth_Y(0, 0) = cos(rAngle);
	_Rotate_wth_Y(1, 0) = 0.0;
	_Rotate_wth_Y(2, 0) = -sin(rAngle);

	_Rotate_wth_Y(0, 1) = 0.0;
	_Rotate_wth_Y(1, 1) = 1.0;
	_Rotate_wth_Y(2, 1) = 0.0;

	_Rotate_wth_Y(0, 2) = sin(rAngle);
	_Rotate_wth_Y(1, 2) = 0.0;
	_Rotate_wth_Y(2, 2) = cos(rAngle);

	return(_Rotate_wth_Y);
}

Matrix3d TorqueJointSpaceControllerRRT::Rotate_with_Z(const double rAngle)
{
	Matrix3d _Rotate_wth_Z(3, 3);

	_Rotate_wth_Z(0, 0) = cos(rAngle);
	_Rotate_wth_Z(1, 0) = sin(rAngle);
	_Rotate_wth_Z(2, 0) = 0.0;

	_Rotate_wth_Z(0, 1) = -sin(rAngle);
	_Rotate_wth_Z(1, 1) = cos(rAngle);
	_Rotate_wth_Z(2, 1) = 0.0;

	_Rotate_wth_Z(0, 2) = 0.0;
	_Rotate_wth_Z(1, 2) = 0.0;
	_Rotate_wth_Z(2, 2) = 1.0;

	return(_Rotate_wth_Z);
}


void TorqueJointSpaceControllerRRT::commandForceCallback(const geometry_msgs::WrenchConstPtr& msg)
{
  f_star_zero_(0) = msg -> force.x;
  f_star_zero_(1) = msg -> force.y;
  f_star_zero_(2) = msg -> force.z;
  f_star_zero_(3) = msg -> torque.x;
  f_star_zero_(4) = msg -> torque.y;
  f_star_zero_(5) = msg -> torque.z;
}

void TorqueJointSpaceControllerRRT::pegInHoleStateCallback(const std_msgs::Int32ConstPtr& msg)
{
  peg_in_hole_state_ = msg -> data;
}

void TorqueJointSpaceControllerRRT::taskStartCallback(const std_msgs::Bool& msg)
{
  task_start_ = msg.data;
}



} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerRRT,
                       controller_interface::ControllerBase)
