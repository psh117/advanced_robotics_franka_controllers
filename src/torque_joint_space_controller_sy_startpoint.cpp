#include <advanced_robotics_franka_controllers/torque_joint_space_controller_sy_startpoint.h>
#include <cmath>
#include <memory>
#include <iostream>
#include <fstream>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/model.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>

#include "math_type_define.h"

namespace advanced_robotics_franka_controllers
{

bool TorqueJointSpaceControllerSyStartpoint::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  joint0_data = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/joint0_data.txt","w");
  save_data_x = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/save_data_fm.txt","w");   
  save_data_x2 = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/save_data_pr.txt","w");   

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

void TorqueJointSpaceControllerSyStartpoint::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  pos_init_ = transform_init_.translation();	
  ori_init_ = transform_init_.rotation();
  ori_first_state_ = ori_init_;
  tilt_angle_z_ = 3*M_PI/180;
  ori_theta_z_ = 0.0;
  xdot_desired_.setZero();

  check_stop = 0;
  check_contact = false;
  check_orientation_ = false;
  check_spiral_done_ = false;
  check_curved_approach_ = false;
  check_yaw_motion_ = false;

  is_check_orientation_first_ = true;
  is_check_contact_first_ = true;
  is_spiral_motion_first_ = true;
  is_curved_approach_first_ = true;
  is_yaw_motion_first_ = true;
  is_first_ = true;

  rotation_z_direction_ = true;

  finish_time = time - start_time_;

  K_p.setZero(); K_v.setZero();
  for (int i = 0; i < 3; i++)
  {
    K_p(i, i) = 5000.0; K_v(i, i) = 100.0; //7000
  }
  K_p(2, 2) = 5000.0; //5000
  
  rotation_init_ = atan2(ori_init_(1,0),ori_init_(0,0)); // to get X angle with resptec to the grobal frame
  
  rotation_duration_ = 3.0;
  descent_speed_ = -0.005; // 5cm/s

  pin_state_ = 0; //0

  trajectory_time = 5.0;


  move_z = 0.03;
  move_x = 0.005;
  move_y = 0.000;
  move_angle = 10.0* M_PI / 180.0;

  //////////////////
  std::ifstream test_set;
  test_set.open("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/start_point_set_5mm_10degree_2.txt");
  for(int i =0; i<100; i++){
  test_set >> move_z_data[i] >> move_x_data[i] >> move_y_data[i] >> move_angle_data[i];
  //std::cout << move_angle_data[i] << std::endl;
  }
  test_set.close();

  std::ifstream test_num;
  test_num.open("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/test_num.txt");
  test_num >> start_point_num;
  test_num.close();
  //start_point_num = 4;

  move_z = move_z_data[start_point_num];
  move_x = move_x_data[start_point_num];
  move_y = move_y_data[start_point_num];
  move_angle = (move_angle_data[start_point_num]*1000)* M_PI / 180.0;

  std::cout<<"First declare: "<<move_x<<", "<<move_y<<std::endl;

}


void TorqueJointSpaceControllerSyStartpoint::update(const ros::Time& time, const ros::Duration& period) {

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
  q_goal << 0.0, -M_PI/6, 0.0, -2*M_PI/3, 0, M_PI/2, M_PI/4;
  q_desired.setZero();

  ros::Duration simulation_time = time - start_time_;
  Eigen::Matrix<double, 7, 1> tau_cmd;
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation());
  Eigen::Vector6d x_dot_(jacobian*qd);

  jacobian_pos_ = jacobian.block(0, 0, 3, 7);

  //f_sensing = (jacobian * jacobian.transpose()).inverse() * jacobian * (tau_measured - gravity);
  f_sensing = (jacobian * mass_matrix.inverse() * jacobian.transpose()).inverse() * jacobian * mass_matrix.inverse() * (tau_measured - gravity);

////////////////


  // double move_z = 0.03;
  // double move_x = 0.006;
  // double move_y = 0.006;
  // double move_angle = 5 * M_PI / 180.0;

  move_z = move_z_data[start_point_num] / 1000;
  move_x = move_x_data[start_point_num] / 1000;
  move_y = move_y_data[start_point_num] / 1000;
  move_angle = move_angle_data[start_point_num];

  pos_goal_z = 0.0464;

  // move_x = 0.005805;
  // move_y = 0.001267;
  // move_angle = -6.496*M_PI/180;

  //pos_init_(0) = 0.52948;
  //pos_init_(1) = -0.45662;

  if(pin_state_ == 0)
  {
    if (is_first_)
    {
      rotation_start_time_ = time;
      ori_first_state_ = rotation_M;
      pos_first_state_ = position;
      trajectory_time = 2.0;
      is_first_ = false;
      std::cout<<"state is 0"<<std::endl;
    }

    if(time.toSec() > rotation_start_time_.toSec() + trajectory_time)
    {    
      pin_state_ = 1;
      is_first_ = true;
      std::cout << "SPIRAL MOTIN IS DONE" << std::endl;
    }


    x_desired_(0) = pos_init_(0);
    x_desired_(1) = pos_init_(1);
    x_desired_(2) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(2), pos_goal_z, 0, 0);
    
    xdot_desired_.setZero();                                                                                                                                                                                             // in "approach process", z velocity is not "zero"

    delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, ori_init_);

    K_p(0, 0) = 500;
    K_p(1, 1) = 500;
    K_p(2, 2) = 3000;
    
    f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
    m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail<3>(); //100 5
    //m_star_ = keepOrientationPerpenticular(ori_init_, rotation_M, x_dot_, 1.0, time.toSec(), rotation_start_time_.toSec());

    f_star_zero_.head(3) = f_star_;
    f_star_zero_.tail(3) = m_star_;

    tau_cmd = jacobian.transpose() * (f_star_zero_);
  }
  else if(pin_state_ == 1)
  {
    if (is_first_)
    {
      rotation_start_time_ = time;
      trajectory_time = 1.0;
      ori_first_state_ = rotation_M;
      pos_first_state_ = position;
      is_first_ = false;
      std::cout<<"state is 1"<<std::endl;
    }

    if(time.toSec() > rotation_start_time_.toSec() + trajectory_time)
    {    
      pin_state_ = 2;
      is_first_ = true;
      std::cout << "SPIRAL MOTIN IS DONE" << std::endl;
    }


    x_desired_(0) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(0), pos_init_(0) + move_x, 0, 0);
    x_desired_(1) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(1), pos_init_(1) + move_y, 0, 0);
    x_desired_(2) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(2), pos_goal_z, 0, 0);
    
    xdot_desired_.setZero();                                                                                                                                                                                             // in "approach process", z velocity is not "zero"

    delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, ori_first_state_);

    K_p(0, 0) = 5000;
    K_p(1, 1) = 5000;
    K_p(2, 2) = 3000;

    f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
    m_star_ = (1.0) * 300 * delphi_delta - 5 * x_dot_.tail<3>(); //100 5

    f_star_zero_.head(3) = f_star_;
    f_star_zero_.tail(3) = m_star_;

    tau_cmd = jacobian.transpose() * (f_star_zero_);
  }
  else if(pin_state_ == 2)
  {
    if (is_first_)
    {
      rotation_start_time_ = time;
      trajectory_time = 1.0;
      ori_first_state_ = rotation_M;
      pos_first_state_ = position;
      is_first_ = false;
      std::cout<<"state is 2"<<std::endl;
    }

    if(time.toSec() > rotation_start_time_.toSec() + trajectory_time)
    {    
      //pin_state_ = 3;
      //is_first_ = true;
      //std::cout << "SPIRAL MOTIN IS DONE" << std::endl;
    }


    // x_desired_(0) = pos_init_(0) + move_x;
    // x_desired_(1) = pos_init_(0) + move_y;
    // x_desired_(2) = pos_init_(2) + move_z;

    x_desired_(0) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(0), pos_init_(0) + move_x, 0, 0);
    x_desired_(1) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(1), pos_init_(1) + move_y, 0, 0);
    x_desired_(2) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(2), pos_goal_z, 0, 0);
    
    xdot_desired_.setZero();
    
    ori_theta_z_ = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, 0, move_angle, 0, 0);
    rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;
    //rotation_z_theta_ << 1, 0, 0, 0, cos(-ori_theta_z_), -sin(-ori_theta_z_), 0, sin(-ori_theta_z_), cos(-ori_theta_z_); //x
    //rotation_z_theta_ << cos(ori_theta_z_), 0, sin(ori_theta_z_), 0, 1, 0, -sin(ori_theta_z_), 0, cos(ori_theta_z_); //y
    target_rotation_ = ori_first_state_*rotation_z_theta_; //EE
  
    delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

    f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
    m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail<3>(); //100 5

    f_star_zero_.head(3) = f_star_;
    f_star_zero_.tail(3) = m_star_;

    tau_cmd = jacobian.transpose() * (f_star_zero_);
  }




/////////////////////////////////////////////////////////////////////
  //     }
  // }

  //tau_cmd.setZero();
  Eigen::Vector3d euler_angle = DyrosMath::rot2Euler(ori_init_.inverse() * rotation_M);

/////////////////////////////////////////////////////////////////////

  // pin_state_ = 10;

  //   f_star_zero_.setZero();

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, ori_init_);
  //   m_star_ = (1.0) * 300 * delphi_delta - 5 * x_dot_.tail(3);//100 5

  //   //f_star_zero_.head(3) = f_star_;
  //   //f_star_zero_(2) = -6.0;
  //   f_star_zero_.tail(3) = m_star_;

  //   f_star_zero_(2) = -15.0;
  //   f_star_zero_(3) = 0.0;
  //   f_star_zero_(4) = 0.0;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_); 


  //   rotation_z_theta_real_ = ori_init_.inverse() * rotation_M;
  //   ori_theta_z_real_ = acos(rotation_z_theta_real_(0));

////////////////////////////////////////////////////
  
  if (print_rate_trigger_()) {
    ROS_INFO("--------------------------------------------------");
    //ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    ROS_INFO_STREAM("time :"<< simulation_time);
    ROS_INFO_STREAM("x_curent : "<< position.transpose());
    ROS_INFO_STREAM("x_desired : "<< x_desired_.transpose());
    //ROS_INFO_STREAM("mass :" << mass_matrix);
    //ROS_INFO_STREAM("ori_theta_z :"<< ori_theta_z_*180/M_PI);
    //ROS_INFO_STREAM("ori_theta_z_real_ :"<< ori_theta_z_real_*180/M_PI);
    ROS_INFO_STREAM("f_sensing : "<< f_sensing.transpose());
    //ROS_INFO_STREAM("spiral_force_ : "<< spiral_force_);
    //ROS_INFO_STREAM("finish_time : "<< finish_time);
    //ROS_INFO_STREAM("delphi_delta : "<< delphi_delta.transpose()*180/M_PI);
    ROS_INFO_STREAM("start_point_num : "<< start_point_num);
    ROS_INFO_STREAM("pin_state_ : "<< pin_state_);
    
    //ROS_INFO_STREAM("euler_angle : "<< euler_angle.transpose());
    ROS_INFO_STREAM("move z : "<< move_z*1000);
    ROS_INFO_STREAM("move x : "<< move_x*1000);
    ROS_INFO_STREAM("move y : "<< move_y*1000);
    ROS_INFO_STREAM("move angle : "<< move_angle/M_PI*180);
  }

  //fprintf(joint0_data, "%lf\t %lf\t\n", ori_theta_z_, ori_theta_z_real_);
  //fprintf(save_data_x2, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", position(0), position(1), position(2), euler_angle(0), euler_angle(1), euler_angle(2));
  //fprintf(save_data_x, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_sensing(0), f_sensing(1), f_sensing(2), f_sensing(3), f_sensing(4), f_sensing(5));
  //fprintf(save_data_x2, "%lf  \t %lf\t %lf\t %lf\t %lf\t %lf\t\n", x_dot_(0), x_dot_(1), x_dot_(2), x_dot_(3), x_dot_(4), x_dot_(5));
 
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

}


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerSyStartpoint,
                       controller_interface::ControllerBase)
