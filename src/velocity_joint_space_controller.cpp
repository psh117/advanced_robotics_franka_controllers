
#include <advanced_robotics_franka_controllers/velocity_joint_space_controller.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include "math_type_define.h"

namespace advanced_robotics_franka_controllers
{

bool VelocityJointSpaceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  // joint_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint_data.txt","w");  
  joint_cmd = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/joint_cmd.txt","w");   
  joint_real = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/joint_real.txt","w");     

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

  /*auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
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
  }*/

  auto* velocity_joint_interface = robot_hw->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting velocity joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(velocity_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  return true;
}

void VelocityJointSpaceController::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());




  q_path.open(file_path+"simple_path_test.txt");
  
  std::string s;

  int index = 0;

  while(!q_path.eof())
  {
    Eigen::Vector7d teaching_joint_data;

    for(int i=0; i<7; i++)
    {
      q_path >> teaching_joint_data(i);
    }

    teaching_joint_datas.push_back(teaching_joint_data);

    //save_data1.save(teaching_joint_data.transpose());
  }

  data_size = (teaching_joint_datas.size()/2) -2;

  last_data = teaching_joint_datas[data_size];

  q_path.close();

}


void VelocityJointSpaceController::update(const ros::Time& time, const ros::Duration& period) {
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

  Eigen::Matrix<double , 7, 1> qd_goal;
  
  q_goal.setZero();
  q_goal_.setZero();
  // q_goal << 0,0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
  q_desired.setZero();

  for(int i = 0; i < 7; i++)
  {
    q_goal(i) = q_init_(i);
    if( i == 5) q_goal(i) = q_init_(i)+M_PI/3;
  }
	
  ros::Duration simulation_time = time - start_time_;
  Eigen::Matrix<double, 7, 1> tau_cmd;
  Eigen::Matrix<double, 7, 1> qd_cmd;
  qd_cmd.setZero();
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation());

  double trajectory_time = 20.0;
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
  //tau_cmd = mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd)) + coriolis;

  bool positive_dir = false;
  int joint_number_vel = 6;
  double vel_deg = 10.0;

  // if(positive_dir == true){
  //   if(q(joint_number_vel) <= 90.0*3.1415926535/180.0){
  //     qd_cmd(joint_number_vel) = vel_deg*3.1415926535/180.0;
  //   }
  //   else{
  //     qd_cmd.setZero();
  //   }
  // }
  // else if(positive_dir == false){
  //   if(q(joint_number_vel) >= 0.0){
  //     qd_cmd(joint_number_vel) = -vel_deg*3.1415926535/180.0;
  //   }
  //   else{
  //     qd_cmd.setZero();
  //   }
  // }
  
  q_goal_ = q;
  q_goal_next = q_desired;

  // if (j < data_size-2){
  //   q_goal_ = teaching_joint_datas[j];
  //   q_goal_next = teaching_joint_datas[j+1];
  // }
  // else{
  //   q_goal_ = last_data;
  //   q_goal_next = last_data;
  // }

  qd_goal = (q_goal_next - q_goal_) * 1000;

  qd_cmd = qd_goal;


  if (print_rate_trigger_()) {
    ROS_INFO("--------------------------------------------------");
    // ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    ROS_INFO_STREAM("time :"<< simulation_time);
    ROS_INFO_STREAM("q_curent : "<< q.transpose());
    ROS_INFO_STREAM("q_desired : "<< q_desired.transpose());
    ROS_INFO_STREAM("qd_cmd : "<< qd_cmd.transpose());


  }

  // fprintf(joint_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", time.toSec(), q_desired(joint_number_vel), q(joint_number_vel), qd(joint_number_vel), tau_cmd(joint_number_vel), tau_J_d(joint_number_vel), tau_measured(joint_number_vel), mass_matrix(joint_number_vel, joint_number_vel));
  fprintf(joint_cmd, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", q_desired(0), q_desired(1), q_desired(2), q_desired(3), q_desired(4), q_desired(5), q_desired(6));
  fprintf(joint_real, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", q(0), q(1), q(2), q(3), q(4), q(5), q(6));
  for (size_t i = 0; i < 7; ++i) {
    //joint_handles_[i].setCommand(tau_cmd(i));
    joint_handles_[i].setCommand(qd_cmd(i));
  }





    if (j < data_size-2)
  {
    j++;
    //j = teaching_joint_datas.size() -1;
  }

}


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::VelocityJointSpaceController,
                       controller_interface::ControllerBase)
