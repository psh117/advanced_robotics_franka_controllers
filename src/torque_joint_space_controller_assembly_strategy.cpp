#include <advanced_robotics_franka_controllers/torque_joint_space_controller_assembly_strategy.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/model.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>

#include "math_type_define.h"

using namespace DyrosMath;

namespace advanced_robotics_franka_controllers
{

bool TorqueJointSpaceControllerAssemblyStrategy::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  //joint0_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint0_data.txt","w");
  save_data_x = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/save_data_fm.txt","w");   
  save_data_x2 = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/save_data_pr.txt","w");   

	std::vector<std::string> joint_names;
  std::string arm_id;
  assem = 0;


 gripper_ac_close.waitForServer();
  gripper_ac_open.waitForServer();


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

  current_pose_pub_ = node_handle.advertise<geometry_msgs::Pose>("/franka_states/current_pose",1);
  current_twist_pub_ = node_handle.advertise<geometry_msgs::Twist>("/franka_states/current_twist",1);
  current_wrench_pub_ = node_handle.advertise<geometry_msgs::Wrench>("/franka_states/current_wrench",1);


  planned_trajectory_ = node_handle.subscribe("/planned_arm_trajectory", 100, &TorqueJointSpaceControllerAssemblyStrategy::trajectoryCallback, this);
  planned_done_ = node_handle.subscribe("/rrt_planned_done", 100, &TorqueJointSpaceControllerAssemblyStrategy::planDoneCallback, this);
  gripper_close_sub_ = node_handle.subscribe("/gripper_close", 100, &TorqueJointSpaceControllerAssemblyStrategy::gripperCloseCallback, this);
  gripper_open_sub_ = node_handle.subscribe("/gripper_open", 100, &TorqueJointSpaceControllerAssemblyStrategy::gripperOpenCallback, this);


  f_star_zero_sub_ = node_handle.subscribe("/f_star_zero_cmd", 100, &TorqueJointSpaceControllerAssemblyStrategy::commandForceCallback, this);
  peg_in_hole_state_sub_ = node_handle.subscribe("/peg_in_hole_state",100, &TorqueJointSpaceControllerAssemblyStrategy::pegInHoleStateCallback, this);
  task_start_sub_ = node_handle.subscribe("/task_start", 100, &TorqueJointSpaceControllerAssemblyStrategy::taskStartCallback, this);
  return true;
}

void TorqueJointSpaceControllerAssemblyStrategy::starting(const ros::Time& time) {
  //start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  // pos_init_ = transform_init_.translation();	
  // ori_init_ = transform_init_.rotation();

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
  for(size_t i = 0; i < 3; i++)
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

  desired_position_.setZero();
  desired_rotation_M.setZero();
  delphi_delta_.setZero();

  Kp_.setIdentity();
  Kd_.setIdentity();

  for (int i = 0; i < 7; i++)
  {
    Kp_(i, i) = 900.0;
    Kd_(i, i) = 10.0;
  }
  Kp_(6, 6) = 300.0;
  Kd_(6, 6) = 5.0;

  task_start_ = false;
  planned_done = false;
  gripper_close = false;
  gripper_open = false;

  q_traj_ = q_init_;


}


void TorqueJointSpaceControllerAssemblyStrategy::update(const ros::Time& time, const ros::Duration& period) {

  const franka::RobotState &robot_state = state_handle_->getRobotState();
  const std::array<double, 42> &jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
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
  q_goal << 0.0, -M_PI/6, 0.0, -2*M_PI/3, 0, M_PI/2, M_PI/4;
  q_desired.setZero();

  Eigen::Matrix<double, 7, 1> tau_cmd;
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation());
  Eigen::Matrix<double, 3, 3> rotation_franka_;
  Eigen::Vector6d x_dot_(jacobian*qd);
  
	rotation_franka_ = rotateWithZ(q(0))*rotateWithY(q(1))*rotateWithY(q(2))*rotateWithY(-q(3))*rotateWithY(q(4))*rotateWithY(-q(5))*rotateWithZ(-(q(6)))*rotateWithZ(-M_PI/4.0);

  Eigen::Vector3d angle_franka = DyrosMath::rot2Euler(rotation_franka_);
  Eigen::Vector3d angle_self_cal = DyrosMath::rot2Euler(rotation_M);

  //f_sensing = (jacobian * jacobian.transpose()).inverse() * jacobian * (tau_measured - gravity);
  f_sensing = (jacobian * mass_matrix.inverse() * jacobian.transpose()).inverse() * jacobian * mass_matrix.inverse() * (tau_measured - gravity);

  
  current_velocity_ = x_dot_;
  current_position_ = position;
  Eigen::Quaterniond current_quat_(rotation_M);


  current_pose_info_.position.x = current_position_(0);
  current_pose_info_.position.y = current_position_(1);
  current_pose_info_.position.z = current_position_(2);

  current_pose_info_.orientation.x = current_quat_.x();
  current_pose_info_.orientation.y = current_quat_.y();
  current_pose_info_.orientation.z = current_quat_.z();
  current_pose_info_.orientation.w = current_quat_.w();

  current_twist_info_.linear.x = current_velocity_(0);
  current_twist_info_.linear.y = current_velocity_(1);
  current_twist_info_.linear.z = current_velocity_(2);

  current_twist_info_.angular.x = current_velocity_(3);
  current_twist_info_.angular.y = current_velocity_(4);
  current_twist_info_.angular.z = current_velocity_(5);

  current_wrench_info_.force.x = f_sensing(0);
  current_wrench_info_.force.y = f_sensing(1);
  current_wrench_info_.force.z = f_sensing(2);
  current_wrench_info_.torque.x = f_sensing(3);
  current_wrench_info_.torque.y = f_sensing(4);
  current_wrench_info_.torque.z = f_sensing(5);

  current_pose_pub_.publish(current_pose_info_); //linear + angular
  current_twist_pub_.publish(current_twist_info_);
  current_wrench_pub_.publish(current_wrench_info_);

  ROS_INFO_STREAM("pos :" << q.transpose());
  // mode change -> q_traj 


//   if (!planned_done)
//   {
//     q_traj_ = q;
//   }
//   tau_cmd = (Kp_ * (q_traj_ - q) - Kd_ * qd); // + coriolis;
  if (planned_done)
  {
    assem = 1;
  }
  else if (task_start_)
  {
    assem = 2;    
  }
  else
  {
    assem = 0;
    q_traj_ = q;
  }

  switch (assem)
  {
    case 1:
      tau_cmd = (Kp_ * (q_traj_ - q) - Kd_ * qd);
      break;

    case 2:
      tau_cmd = jacobian.transpose() * (f_star_zero_);
      break;

    case 0:
      tau_cmd.setZero();
      break;

  }
  
  // if (task_start_)
  // {
  //   tau_cmd = jacobian.transpose() * (f_star_zero_);
  // }
  // else if (planned_done)
  // {
  //   tau_cmd = (Kp_ * (q_traj_ - q) - Kd_ * qd); // + coriolis;
  // }
  // else
  // {
  //   tau_cmd.setZero();
  // }

  //tau_cmd = jacobian.transpose() * (f_star_zero_);
 // tau_cmd.setZero();
  // if (print_rate_trigger_()) {
  //  // ROS_INFO("--------------------------------------------------");
  //  ROS_INFO_STREAM("q_traj_ : "<< q_traj_.transpose());
  // }


    // ROS_INFO("--------------------------------------------------");
    // ROS_INFO_STREAM("f_star_zero_ : "<< f_star_zero_.transpose());
    
    


  fprintf(save_data_x2, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", rotation_M(0,0), rotation_M(0,1), rotation_M(0,2), rotation_M(1,0), rotation_M(1,1), rotation_M(1,2),rotation_M(2,0),rotation_M(2,1),rotation_M(2,2));
  //fprintf(save_data_x, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_sensing(0), f_sensing(1), f_sensing(2), f_sensing(3), f_sensing(4), f_sensing(5));
  fprintf(save_data_x, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", rotation_franka_(0,0), rotation_franka_(0,1), rotation_franka_(0,2), rotation_franka_(1,0), rotation_franka_(1,1), rotation_franka_(1,2),rotation_franka_(2,0),rotation_franka_(2,1),rotation_franka_(2,2));
  //fprintf(save_data_x, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", angle_franka(0), angle_franka(1), angle_franka(2), angle_self_cal(0), angle_self_cal(1), angle_self_cal(2));
 
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }

  if (gripper_close)
  {
    epsilon.inner = 0.05;//0.005;
    epsilon.outer = 0.05;//0.005;
    close_goal.speed = 0.1;
    close_goal.width = 0.005;//0.003;
    close_goal.force = 100.0;
    close_goal.epsilon = epsilon;
    gripper_ac_close.sendGoal(close_goal);
    gripper_close = false;

  }

  if (gripper_open)
  {
    open_goal.speed = 0.1;
    open_goal.width = 0.08;
    gripper_ac_open.sendGoal(open_goal);
    gripper_open = false;

    // gripper_open = false;
    // epsilon.inner = 0.005;
    // epsilon.outer = 0.005;
    // close_goal.speed = 0.1;
    // close_goal.width = 0.1;
    // close_goal.force = 1.0;
    // close_goal.epsilon = epsilon;
    // gripper_ac_close.sendGoal(close_goal);
  }
    // gripper_close = false;
    // open_goal.speed = 0.1;
    // open_goal.width = 0.08;

    // gripper_ac_open.sendGoal(open_goal);
    // gripper_open = false;
    // std::cout <<"2" <<std::endl;
 
}

void TorqueJointSpaceControllerAssemblyStrategy::commandForceCallback(const geometry_msgs::WrenchConstPtr& msg)
{
  f_star_zero_(0) = msg -> force.x;
  f_star_zero_(1) = msg -> force.y;
  f_star_zero_(2) = msg -> force.z;
  f_star_zero_(3) = msg -> torque.x;
  f_star_zero_(4) = msg -> torque.y;
  f_star_zero_(5) = msg -> torque.z;
}

void TorqueJointSpaceControllerAssemblyStrategy::pegInHoleStateCallback(const std_msgs::Int32ConstPtr& msg)
{
  peg_in_hole_state_ = msg -> data;
}

void TorqueJointSpaceControllerAssemblyStrategy::taskStartCallback(const std_msgs::Bool& msg)
{
  task_start_ = msg.data;
  if (task_start_)
    planned_done = false;
  
}

void TorqueJointSpaceControllerAssemblyStrategy::trajectoryCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for (size_t i = 0; i < msg->position.size(); i++)
    {
        q_traj_[i] = msg->position[i];
    }
}


void TorqueJointSpaceControllerAssemblyStrategy::planDoneCallback(const std_msgs::Bool& msg)
{
    planned_done = msg.data;
}

void TorqueJointSpaceControllerAssemblyStrategy::gripperCloseCallback(const std_msgs::Bool& msg)
{
    gripper_close = msg.data;
    gripper_open = false;
}

void TorqueJointSpaceControllerAssemblyStrategy::gripperOpenCallback(const std_msgs::Bool& msg)
{

    gripper_open = msg.data;
    gripper_close = false;
}




} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerAssemblyStrategy,
                       controller_interface::ControllerBase)
