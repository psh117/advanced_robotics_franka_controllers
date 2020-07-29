#include <advanced_robotics_franka_controllers/torque_joint_space_controller.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/model.h>

#include "math_type_define.h"

namespace advanced_robotics_franka_controllers
{

bool TorqueJointSpaceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  joint0_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint0_data.txt","w");  

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

void TorqueJointSpaceController::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  pos_init_ = transform_init_.translation();	
  ori_init_ = transform_init_.rotation();
   
}


void TorqueJointSpaceController::update(const ros::Time& time, const ros::Duration& period) {

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
  q_goal << 0, 0.0, 0.0, -M_PI/2, 0, M_PI/2, 0;
  //q_goal << M_PI/6, M_PI/6, M_PI/6, -M_PI/6, M_PI/6, M_PI/6, M_PI/6;
  //q_goal << 0, -M_PI/6, 0, -2*M_PI/3, 0, M_PI/2, M_PI/4;
  q_desired.setZero();

  //q_goal = q_init_;
  //q_goal(5) -= M_PI/6.0;

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


  qd_desired.setZero();

  // double amplitude = 60.0*3.1415926535/180.0;
  // double frequency = 0.075*(1.0-cos((1.0/250.0)*(time.toSec() - start_time_.toSec())));

  // bool positive_dir = false;
  // double qdot_d = 10.0*3.1415926535/180.0;;
  // int joint_number = 6;
  // if(positive_dir == true){
  //   q_desired(joint_number) = q_init_(joint_number) + qdot_d*(time.toSec() - start_time_.toSec());
  //   if(q(joint_number) >= 90.0*3.1415926535/180.0 || q_desired(joint_number) >= 90.0*3.1415926535/180.0){
  //     q_desired(joint_number) = 90.0*3.1415926535/180.0;
  //   }
  // }
  // else if(positive_dir == false){
  //   qdot_d = -qdot_d;
  //   q_desired(joint_number) = q_init_(joint_number) + qdot_d*(time.toSec() - start_time_.toSec());
  //   if(q(joint_number) <= 0.0*3.1415926535/180.0 || q_desired(joint_number) <= 0.0*3.1415926535/180.0){
  //     q_desired(joint_number) = 0.0*3.1415926535/180.0;
  //   }
  // }

  //q_desired(0) = q_init_(0) + amplitude*(1.0-cos(2.0*3.1415926535*frequency*(time.toSec() - start_time_.toSec())));
  //q_desired(0) = q_init_(0) + amplitude*(1.0-cos(2.0*3.1415926535*frequency*(time.toSec() - start_time_.toSec())));
  //qd_desired(0) = 2.0*3.1415926535*frequency*amplitude*sin(2.0*3.1415926535*frequency*(time.toSec() - start_time_.toSec()));
  

  double kp, kv;
  kp = 100;
  kv = 5;

  Eigen::Matrix<double , 7, 7> A_diag;
  A_diag.setIdentity();

  for(int i=0; i<7; i++){
    A_diag(i, i) = mass_matrix(i, i);
  }


//tau_cmd =  mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd));
tau_cmd =  ( 800*(q_desired - q) + 5*(qd_desired - qd));

////  tau_cmd =  mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd));// + coriolis;
  //tau_cmd = -gravity + graivity_mod;
  //tau_cmd.setZero();

  if (print_rate_trigger_()) {
    ROS_INFO("--------------------------------------------------");
    ROS_INFO_STREAM("tau :" << gravity.transpose());
    //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    ROS_INFO_STREAM("time :"<< simulation_time);
    ROS_INFO_STREAM("q_curent : "<< q.transpose());
    ROS_INFO_STREAM("q_desired : "<< q_desired.transpose());
    ROS_INFO_STREAM("mass_matrix : "<<"\n"<< mass_matrix);

  }

//  fprintf(joint0_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", time.toSec(), q_desired(0), q(0), qd(0), tau_cmd(0), tau_J_d(0), tau_measured(0), gravity);
    fprintf(joint0_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", gravity(0), gravity(1), gravity(2), gravity(3), gravity(4), gravity(5), gravity(6));

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

}


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceController,
                       controller_interface::ControllerBase)
