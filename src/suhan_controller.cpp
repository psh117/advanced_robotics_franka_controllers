
#include <advanced_robotics_franka_controllers/suhan_controller.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include "math_type_define.h"

namespace advanced_robotics_franka_controllers
{

void SuhanController::initTasks()
{
  // BECAUSE OF THE COMPILE ERROR, I REPLACED ros::Time to double
  tasks_.push_back(std::make_pair(3.0, ControlType::PathFollowing));
  tasks_.push_back(std::make_pair(7.0, ControlType::Assembly1));
  tasks_.push_back(std::make_pair(12.0, ControlType::PathFollowing));
  tasks_.push_back(std::make_pair(15.0, ControlType::Assembly2));
}

void SuhanController::getTrajectories()
{

}


bool SuhanController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
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
  initTasks();
  return true;
}

void SuhanController::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }

  time_starting_assembly_ = 0;
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  
}


void SuhanController::update(const ros::Time& time, const ros::Duration& period) {
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  const std::array<double, 42> &jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  const std::array<double, 7> &gravity_array = model_handle_->getGravity();
  const std::array<double, 49> &massmatrix_array = model_handle_->getMass();
  const std::array<double, 7> &coriolis_array = model_handle_->getCoriolis();



  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(massmatrix_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> qd(robot_state.dq.data());

  Eigen::Matrix<double , 6, 7> jacobian_euler;
  Eigen::Matrix<double , 12, 7> jacobian_dc;
  Eigen::Matrix<double , 7, 1> q_goal;
  Eigen::Matrix<double , 7, 1> q_desired;
  Eigen::Matrix<double , 7, 1> qd_desired;
  Eigen::Matrix<double , 12, 1> x_goal; 
  Eigen::Matrix<double , 12, 1> x_desired;
  Eigen::Matrix<double , 12, 1> x_current;
  
  q_goal.setZero();
  q_goal << 0,0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
  q_desired.setZero();
	
  ros::Duration simulation_time = time - start_time_;
  Eigen::Matrix<double, 7, 1> tau_cmd;
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation());

  double trajectory_time = 5.0;
  for(int i=0; i<7;i++)
  {
    q_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
                                        q_init_(i), q_goal(i), 0, 0);
    qd_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
                                        q_init_(i), q_goal(i), 0, 0);
  }
  double kp, kv;

  kp = 1500;
  kv = 10;
  switch (time2task(time))
  {
  case ControlType::PathFollowing:
    tau_cmd = movePathUpdate(time,Eigen::Matrix<double, 7,1>::Zero());
    break;
  case ControlType::Assembly1:
  case ControlType::Assembly2:
   /*  if(time_starting_assembly_ == 0)
    {
      spiral_starting_pos_assembly_ = position;
      ori_init_assembly_ = rotation_M;
      check_stop_assemlby_ = 0;
      time_starting_assembly_ = 1;
    }
    tau_cmd = assembleUpdate(time, ControlType::Assembly2);
    */

    break;
  }

  tau_cmd = mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd)) + coriolis;

  if (print_rate_trigger_()) {
    ROS_INFO("--------------------------------------------------");
    ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    ROS_INFO_STREAM("time :"<< simulation_time);
    ROS_INFO_STREAM("q_curent : "<< q.transpose());
    ROS_INFO_STREAM("q_desired : "<< q_desired.transpose());


  }

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }

}


Eigen::Matrix<double, 7, 1> SuhanController::movePathUpdate(const ros::Time& time, Eigen::Matrix<double, 7, 1> target_pos)
{

  Eigen::Matrix<double, 7, 1> tau_cmd;
  //tau_cmd = mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd)) + coriolis;
  return tau_cmd;
}

Eigen::Matrix<double, 7, 1> SuhanController::assembleUpdate(const ros::Time& time, SuhanController::ControlType assembly_type)
{
  /*
  // if(time - task_start_time_)
  Eigen::Matrix<double, 7, 1> tau_cmd;
  Eigen::Vector3d x_desired_;
  Eigen::Vector3d pos_hole_; //the location of a hole(the position at last time of spiral)
  Eigen::Matrix3d K_p; //control gain for peg in hole task
  Eigen::Matrix3d K_v; //control gain for peg in hole task
  Eigen::Vector3d f_star_; //linear force
	Eigen::Vector3d m_star_; //orientation
	Eigen::Matrix<double, 6, 1> f_star_zero_;
  Eigen::Matrix<double, 6, 1> f_sensing;
  Eigen::Vector3d delphi_delta; // value related with orientation

  x_desired_.setZero();

  //TODO : how to define "position" later
  if ((position(2) < spiral_starting_pos_assembly_ (2)-0.008)&&(check_stop_assemlby_ == 0)) //To check a pin is inserted correctly
  {
    check_stop_assemlby_ = 1;
    pos_hole_ = position; // store the location of a hole
  }

  x_desired_.block<2, 1>(0, 0) = DyrosMath::spiral(time.toSec(), start_time_.toSec(), start_time_.toSec() + 40, spiral_starting_pos_assembly_.block<2, 1>(0, 0), 0.005, 0.002);
	x_desired_(2) = spiral_starting_pos_assembly_(2);

  if(check_stop_assemlby_ == 1) // if a peg is inserted
  {
    x_desired_ = pos_hole_;
  }

  //////"for loop" to set gains------------
  K_p.setZero(); K_v.setZero();
	for (int i = 0; i < 3; i++)
	{
		K_p(i, i) = 5000.0; K_v(i, i) = 100.0; //7000
	}
  K_p(2, 2) = 3000.0; //5000
  ///------------------------------------
  

  // TODO : how to define "x_dot_" later
  // TODO : how to define "rotation_M" later
  f_star_ = K_p * (x_desired_ - position) - K_v * x_dot_.head(3));  
  f_star_(2) = -6.0; // setpoint torque by trial

  delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, ori_init_assembly_);
  m_star_ = (1.0) * 200 * delphi_delta - 20 * x_dot_.tail(3);
  
 
  f_star_zero_.head(3) = f_star_;
	f_star_zero_.tail(3) = m_star_;

  // TODO : how to define "tau_measured" later
  // TODO : how to define "gravity" later
  // TODO : how to define "jacobian" later
  f_sensing = (jacobian * jacobian.transpose()).inverse() * jacobian * (tau_measured - gravity);

  tau_cmd = jacobian.transpose() * f_star_zero_;

  return tau_cmd;

  */
}


SuhanController::ControlType SuhanController::time2task(const ros::Time& time)
{
  if(task_index_ < tasks_.size())
  {
    if (time.toSec() > tasks_[task_index_].first)
    {
      task_index_ ++;
      task_start_time_ = time;
      // TODO: init_pos update
    }

    return tasks_[task_index_].second;

  }
  return ControlType::None;
}


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::SuhanController,
                       controller_interface::ControllerBase)
