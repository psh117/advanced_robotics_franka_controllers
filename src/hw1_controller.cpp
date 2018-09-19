#include <advanced_robotics_franka_controllers/hw1_controller.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace advanced_robotics_franka_controllers
{

bool HW1Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
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
  return true;
}

void HW1Controller::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  pos_init_ = transform_init_.translation();	
  ori_init_ = transform_init_.rotation();
  
}


void HW1Controller::update(const ros::Time& time, const ros::Duration& period) {
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
	
  ros::Duration simulation_time = time - start_time_;
  // Compute here

  Eigen::Matrix<double, 6, 6> lamda;
  Eigen::Matrix<double, 3, 3> lamda_pos;
  Eigen::Matrix<double, 3, 3> lamda_ori;

  Eigen::Matrix<double, 7, 1> tau_joint_sensor;

  tau_joint_sensor = tau_measured - gravity;
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation());
  
  Eigen::Matrix<double, 3, 7> J_pos(jacobian.topRows(3));
  Eigen::Matrix<double, 3, 7> J_ori(jacobian.bottomRows(3));

  lamda = (jacobian * mass_matrix.inverse() * jacobian.transpose()).inverse();
  lamda_pos = lamda.block(0,0,3,3);
  lamda_ori = lamda.block(3,3,3,3);
  
 
  Eigen::Matrix<double, 7, 1> tau_cmd;
	
	
  // Eigen::Matrix<doubtau_joint_sensor
  // Eigen::Vector3d e_tau_joint_sensor
  // Eigen::Vector3d e_tau_joint_sensor
  // Eigen::VectorXd e_tau_joint_sensor
  // e_total.resize(6);tau_joint_sensor
  // e_total.setZero();

  // rotation_error = rotation_M.transpose() * ori_init_;


  // e_rot_ee(0) = rotation_error(2,1) - rotation_error(1,2);
  // e_rot_ee(1) = rotation_error(0,2) - rotation_error(2,0);
  // e_rot_ee(2) = rotation_error(1,0) - rotation_error(0,1);

  // e_rot = rotation_M * e_rot_ee;  


  // e_total(0) = (pos_init_ - position)(0);
  // e_total(1) = (pos_init_ - position)(1);
  // e_total(2) = (pos_init_ - position)(2);
  // e_total(3) = e_rot(0);
  // e_total(4) = e_rot(1);
  // e_total(5) = e_rot(2);  
  

	// int kp_joint = 30;
  // Eigen::Matrix<double, 6 , 6> kp_oper;
  // kp_oper.setZero();
  // kp_oper(0,0) = kp_operation;
  // kp_oper(1,1) = kp_operation;
  // kp_oper(2,2) = kp_operation;
  // kp_oper(3,3) = kp_ori_operation;
  // kp_oper(4,4) = kp_ori_operation;
  // kp_oper(5,5) = kp_ori_operation;
  

  //tau_cmd =  J_pos.transpose() * kp_operation * lamda_pos * (pos_init_ - position) + J_ori.transpose() * kp_ori_operation * lamda_ori * e_rot;
  //tau_cmd =  J_pos.transpose() * kp_operation * lamda_pos * (pos_init_ - position);
  //tau_cmd =  J_pos.transpose() * kp_operation * (pos_init_ - position) + J_ori.transpose() * kp_ori_operation * e_rot;
  //tau_cmd = jacobian.transpose() * lamda * kp_oper * e_total; 
	//tau_cmd = kp_joint*mass_matrix*(q_init_-q);
  //tau_cmd = kp_joint*(q_init_-q);

	
  //

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
  }

  if (print_rate_trigger_()) {
    ROS_INFO("--------------------------------------------------");
    ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    ROS_INFO_STREAM("time :"<< simulation_time);

    

  }
}


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::HW1Controller,
                       controller_interface::ControllerBase)
