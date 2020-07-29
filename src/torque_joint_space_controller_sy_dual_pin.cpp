#include <advanced_robotics_franka_controllers/torque_joint_space_controller_sy_dual_pin.h>
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

bool TorqueJointSpaceControllerSyDualPin::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  joint0_data = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/joint0_data.txt","w");
  save_data_x = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/save_data_fm.txt","w");
  save_data_x2 = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/save_data_pr.txt","w");
  save_data_x3 = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/save_data_daul_time.txt","w");

  reaction_force = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/reaction_force.txt","w");
  save_cmd = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/iros/save_cmd.txt","w");
  save_fm = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/iros/save_fm.txt","w");
  gain_tunning = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/iros/gain_tunning.txt","w");

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

void TorqueJointSpaceControllerSyDualPin::starting(const ros::Time& time) {
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
  is_done_ = false;
  rotation_z_direction_ = true;

  finish_time = time - start_time_;
  strategy_start_time_ = time;
  recovery_start_time_ = time;

  K_p.setZero(); K_v.setZero();
  for (int i = 0; i < 3; i++)
  {
    K_p(i, i) = 7000.0; K_v(i, i) = 100.0; //7000
  }
  K_p(2, 2) = 5000.0; //5000

  rotation_init_ = atan2(ori_init_(1,0),ori_init_(0,0)); // to get X angle with resptec to the grobal frame

  rotation_duration_ = 3.0;
  descent_speed_ = -0.005; // 5cm/s

  pin_state_ = 0; //0 //100

  ori_change_direction = 0;
  ori_check_time = 0;

  trajectory_time = 5.0;
  exp_num = 67;

  for(int i = 0; i<10; i++){
    last_z_pos[i] = 50; //pos_init_(2);
    if(i>4){
      last_z_pos[i] = 100;
    }
  }
  last_z_pos_avr_1 = 0.0;
  last_z_pos_avr_2 = -100.0;

  //move_z = 0.03;
  //move_x = 0.001;
  //move_y = -0.0005;
  //move_angle = 0.605* M_PI / 180.0;

  // std::ifstream gain_setting;
  // gain_setting.open("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/gain_setting.txt");
  // gain_setting >> input_p_gain_ >> input_d_gain_;
  // gain_setting.close();
  


  std::ifstream test_num;
  test_num.open("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/test_velocity.txt");
  test_num >> input_vel_spiral >> input_vel_theta >> input_p_gain_ >> input_d_gain_ >> input_wp_gain_ >> input_wd_gain_;
  test_num.close();


  // std::ifstream test_set;
  // test_set.open("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/test_set.txt");
  // for(int i =0; i<100; i++){
  // test_set >> move_z_data[i] >> move_x_data[i] >> move_y_data[i] >> move_angle_data[i];
  // std::cout << move_angle_data[i] << std::endl;
  // }
  // test_set.close();
  
  theta_spiral_ = 10.0*M_PI/180;

  // pin_state_ =5;
}


void TorqueJointSpaceControllerSyDualPin::update(const ros::Time& time, const ros::Duration& period) {

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
  Eigen::Matrix<double , 6, 6> jacobian_6;

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


  Eigen::Matrix<double, 6, 6> lambda;
  Eigen::Matrix<double, 7, 6> J_bar;
  Eigen::Matrix<double, 6, 1> f_measured;
  Eigen::Vector3d force_ee; //w.r.t end-effector
  Eigen::Vector3d moment_ee;

  lambda = (jacobian*mass_matrix.inverse()*jacobian.transpose()).inverse();
  J_bar = mass_matrix.inverse()*jacobian.transpose()*lambda;
  f_measured = J_bar.transpose()*(tau_measured - gravity); //w.r.t global frame

  force_ee = rotation_M.transpose()*f_measured.head<3>();
  moment_ee = rotation_M.transpose()*f_measured.tail<3>();

////////////////////////////////////////////////////

  // if(pin_state_ == 100)
  // {
  //   if (is_first_)
  //   {
  //     rotation_start_time_ = time;
  //     ori_first_state_ = rotation_M;
  //     pos_first_state_ = position;
  //     trajectory_time = 4.0;
  //     is_first_ = false;
  //     std::cout<<"state is 0"<<std::endl;
  //     //exp_num = exp_num + 1;


  //     recovery_start_time_ = time;
  //     move_z = move_z_data[exp_num];
  //     move_x = move_x_data[exp_num];
  //     move_y = move_y_data[exp_num];
  //     move_angle = move_angle_data[exp_num];

  //     //move_z = 0.03;
  //     //move_x = 0.001;
  //     //move_y = -0.0005;
  //     //move_angle = 0.605* M_PI / 180.0;
  //   }

  //   if(time.toSec() > rotation_start_time_.toSec() + trajectory_time)
  //   {
  //     pin_state_ = 101;
  //     is_first_ = true;
  //     std::cout << "SPIRAL MOTIN IS DONE" << std::endl;
  //   }


  //   x_desired_(0) = pos_init_(0);
  //   x_desired_(1) = pos_init_(1);
  //   x_desired_(2) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(2), pos_init_(2) + move_z, 0, 0);

  //   xdot_desired_.setZero();                                                                                                                                                                                             // in "approach process", z velocity is not "zero"

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, ori_init_);

  //   K_p(0, 0) = 500;
  //   K_p(1, 1) = 500;
  //   K_p(2, 2) = 3000;

  //   f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
  //   m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail<3>(); //100 5
  //   //m_star_ = keepOrientationPerpenticular(ori_init_, rotation_M, x_dot_, 1.0, time.toSec(), rotation_start_time_.toSec());

  //   f_star_zero_.head(3) = f_star_;
  //   f_star_zero_.tail(3) = m_star_;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);
  // }
  // else if(pin_state_ == 101)
  // {
  //   if (is_first_)
  //   {
  //     rotation_start_time_ = time;
  //     trajectory_time = 2.0;
  //     ori_first_state_ = rotation_M;
  //     pos_first_state_ = position;
  //     is_first_ = false;
  //     std::cout<<"state is 1"<<std::endl;
  //   }

  //   if(time.toSec() > rotation_start_time_.toSec() + trajectory_time)
  //   {
  //     pin_state_ = 102;
  //     is_first_ = true;
  //     std::cout << "SPIRAL MOTIN IS DONE" << std::endl;
  //   }


  //   x_desired_(0) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(0), pos_init_(0) + move_x, 0, 0);
  //   x_desired_(1) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(1), pos_init_(1) + move_y, 0, 0);
  //   x_desired_(2) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(2), pos_init_(2) + move_z, 0, 0);

  //   xdot_desired_.setZero();                                                                                                                                                                                             // in "approach process", z velocity is not "zero"

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, ori_first_state_);

  //   K_p(0, 0) = 5000;
  //   K_p(1, 1) = 5000;
  //   K_p(2, 2) = 3000;

  //   f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
  //   m_star_ = (1.0) * 300 * delphi_delta - 5 * x_dot_.tail<3>(); //100 5

  //   f_star_zero_.head(3) = f_star_;
  //   f_star_zero_.tail(3) = m_star_;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);
  // }
  // else if(pin_state_ == 102)
  // {
  //   if (is_first_)
  //   {
  //     rotation_start_time_ = time;
  //     trajectory_time = 2.0;
  //     ori_first_state_ = rotation_M;
  //     pos_first_state_ = position;
  //     ori_theta_z_ = 0.0;
  //     is_first_ = false;
  //     std::cout<<"state is 2"<<std::endl;
  //   }

  //   if(time.toSec() > rotation_start_time_.toSec() + trajectory_time)
  //   {
  //     pin_state_ = 0;
  //     is_first_ = true;
  //     std::cout << "SPIRAL MOTIN IS DONE" << std::endl;
  //   }


  //   // x_desired_(0) = pos_init_(0) + move_x;
  //   // x_desired_(1) = pos_init_(0) + move_y;
  //   // x_desired_(2) = pos_init_(2) + move_z;

  //       x_desired_(0) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(0), pos_init_(0) + move_x, 0, 0);
  //   x_desired_(1) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(1), pos_init_(1) + move_y, 0, 0);
  //   x_desired_(2) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, pos_first_state_(2), pos_init_(2) + move_z, 0, 0);

  //   xdot_desired_.setZero();

  //   ori_theta_z_ = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + trajectory_time, 0, move_angle, 0, 0);
  //   rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;
  //   //rotation_z_theta_ << 1, 0, 0, 0, cos(-ori_theta_z_), -sin(-ori_theta_z_), 0, sin(-ori_theta_z_), cos(-ori_theta_z_); //x
  //   //rotation_z_theta_ << cos(ori_theta_z_), 0, sin(ori_theta_z_), 0, 1, 0, -sin(ori_theta_z_), 0, cos(ori_theta_z_); //y
  //   target_rotation_ = ori_first_state_*rotation_z_theta_; //EE

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

  //   f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
  //   m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail<3>(); //100 5

  //   f_star_zero_.head(3) = f_star_;
  //   f_star_zero_.tail(3) = m_star_;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);
  // }

////////////////////////////////////////////////////

  if(pin_state_ == 0)
  {

    if(is_first_)
    {
      rotation_start_time_ = time;
      strategy_start_time_ = time;
      //x_desired_ = pos_init_;
      x_desired_ = position;
      descent_speed_ = -0.005; //-0.01
      xdot_desired_(2) = descent_speed_;
      contact_force_ = -10; //0.05 // -1
      ori_first_state_ = rotation_M;
      pos_first_state_ = position;
      is_first_ = false;
      //rotation_duration_ = 0.5;
      std::cout<<"state is 0"<<std::endl;
    }

    if (f_measured(2) <= contact_force_) //f_z is changed frome positive value to negative.
    {
      //check_contact = true; // after finishing the whole process, this value must be changed to "false"
      pin_state_ = 2; // contact -> spiral //2
      is_first_ = true;
      approach_time = time - rotation_start_time_;
      //tau_cmd = jacobian.transpose() * (f_star_zero_.setZero());
      std::cout << "CONTACT IS DETECTED" << std::endl;
    }

    // if(last_z_pos_avr_2 >= -0.000001)
    // {
    //   pin_state_ = 2; // contact -> spiral
    //   is_first_ = true;
    //   std::cout << "CONTACT IS DETECTED" << std::endl;
    //   std::cout << last_z_pos_avr_2 << std::endl;
    // }

    for (int i = 0; i < 3; i++)
    {
      K_p(i, i) = 5000.0; K_v(i, i) = 100.0; //7000
    }

    f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head(3));
    m_star_ = keepOrientationPerpenticularOnlyXY(ori_first_state_, rotation_M, x_dot_, 0.4, time.toSec(), strategy_start_time_.toSec());

    f_star_zero_.head(3) = f_star_;
    f_star_zero_.tail(3) = m_star_;

    tau_cmd = jacobian.transpose() * (f_star_zero_);

    x_desired_(2) += descent_speed_ / 1000.0;

//////////////
    for(int i = 0; i<9; i++){
      last_z_pos[9-i] = last_z_pos[8-i];
    }
    last_z_pos[0] = position(2);
    // last_z_pos_avr_1 = 0.0;
    // last_z_pos_avr_2 = 0.0;
    // for(int i = 0; i<5; i++){
    //   last_z_pos_avr_1 = last_z_pos_avr_1 + last_z_pos[i];
    // }
    // for(int i = 5; i<10; i++){
    //   last_z_pos_avr_2 = last_z_pos_avr_2 + last_z_pos[i];
    // }
    // last_z_pos_avr_1 = last_z_pos_avr_1/5;
    // last_z_pos_avr_2 = last_z_pos_avr_2/5;

    last_z_pos_avr_2 = (last_z_pos[0] - last_z_pos[9])/10;

//////////////
  }


  // else if(pin_state_ == 1)
  // {
  //   if (is_check_contact_first_)
  //   {
  //     target_rotation_ = rotation_M; //to keep the initial orientation
  //     x_desired_ = position;         //to fix x,y position
  //     descent_speed_ = -0.02; //-0.01
  //     xdot_desired_(2) = descent_speed_;
  //     contact_force_ = -4; //0.05 // -1
  //     is_check_contact_first_ = false;
  //     std::cout<<"state is 1"<<std::endl;
  //   }

  //   if (f_sensing(2) <= contact_force_) //f_z is changed frome positive value to negative.
  //   {
  //     //check_contact = true; // after finishing the whole process, this value must be changed to "false"
  //     pin_state_ = 2; // contact -> spiral

  //     //pos_init_(2) = position(2);
  //     tau_cmd = jacobian.transpose() * (f_star_zero_.setZero());
  //     std::cout << "CONTACT IS DETECTED" << std::endl;
  //   }

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

  //   f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
  //   m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail<3>(); //100 5

  //   f_star_zero_.head(3) = f_star_;
  //   f_star_zero_.tail(3) = m_star_;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);

  //   x_desired_(2) += descent_speed_ / 1000.0;
  // }
//   else if(pin_state_ == 2)
//   {
//     if (is_first_)
//     {
//       spiral_start_time_ = time;
//       spiral_origin_ = position;
//       // spiral_linear_velocity_ = 0.003; //0.005
//       spiral_linear_velocity_ = input_vel_spiral;
//       spiral_pitch_ = 0.0015; //0.0025 //0.000907
//       spiral_duration_ = 60.0;
//       spiral_force_limit_ = 10; //7
//       force_press_z_ = -10.0; //-10
//       target_rotation_ = rotation_M;
//       ori_first_state_ = rotation_M;
//       pos_first_state_ = position;
//       x_desired_(2) = spiral_origin_(2);
//       ori_theta_z_ = 0.0;
//       ori_duration = 0.1 * 2;
//       //input_vel_theta = 30;
//       ori_duration = (10/input_vel_theta)*2;
//       tilt_angle_z_ = 10*M_PI/180;
//       is_first_ = false;
//       std::cout<<"state is 2"<<std::endl;
//     }

//     spiral_force_ = sqrt(f_measured(0) * f_measured(0) + f_measured(1) * f_measured(1));

//     //if (spiral_force_ >= spiral_force_limit_)
//     if ((spiral_force_ >= spiral_force_limit_)&&(position(2) < pos_first_state_(2)-0.0005))
//     //if(0)
//     {
//       // pin_state_ = 6; //4 //6
//       // is_first_ = true;
//       // ori_check_time = 0;
//       // ori_change_direction = 0;
//       // insert_last_z_pos = position(2);
//       // spiral_time = time - spiral_start_time_;
//       // std::cout << "SPIRAL MOTIN IS DONE" << std::endl;
//       // std::cout << "SPIRAL FORCE IS OVER THE LIMIT" << std::endl;
//       // std::cout << "pos_first_state_(2) : " << pos_first_state_(2) << std::endl;
//       // std::cout << "position(2) : " << position(2) << std::endl;
//     }

//     if (position(2) < pos_first_state_(2)-0.00025)
//     {
//       std::cout << "Z - 1mm" << std::endl;
//       // pin_state_ = 4;
//       // is_first_ = true;
//       // ori_check_time = 0;
//       // ori_change_direction = 0;
//       // insert_last_z_pos = position(2);
//       // std::cout << "SPIRAL MOTIN IS DONE" << std::endl;
//        std::cout<<"search duration: "<<time.toSec() - spiral_start_time_.toSec()<<std::endl;
//     }

//     x_desired_.block<2, 1>(0, 0) = DyrosMath::spiral(time.toSec(), spiral_start_time_.toSec(), spiral_start_time_.toSec() + spiral_duration_, spiral_origin_.block<2, 1>(0, 0), spiral_linear_velocity_, spiral_pitch_); //0.0035 //0.02
//     x_desired_(2) = spiral_origin_(2);
//     xdot_desired_.setZero();                                                                                                                                                                                             // in "approach process", z velocity is not "zero"


//     f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
//     //f_star_(2) = force_press_z_; //-6, -10
//     //f_star_(2) = force_press_z_ + 0.5 * (force_press_z_ - f_sensing(2)) + 100 * (xdot_desired_(2) - x_dot_(2));

// ///////
//     if(ori_change_direction == 0)
//     {
//       if(ori_check_time == 0)
//       {
//         time_ori_0 = time;
//         ori_check_time = 1;
//         std::cout << "check_time" << std::endl;
//       }
//       //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
//       //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

//       //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
//       //target_rotation_ = rotation_f;

//       ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration/2, 0, tilt_angle_z_, 0, 0);

//       rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

//       target_rotation_ = rotation_z_theta_ * ori_first_state_;
//       //target_rotation_ = ori_first_state_ * rotation_z_theta_; //EE

//       if(time.toSec() > time_ori_0.toSec() + ori_duration/2)
//       {
//         ori_change_direction = 1;
//         ori_check_time = 0;
//         // std::cout << "check_time_end" << std::endl;
//       }
//     }
//     if(ori_change_direction == 1)
//     {
//       if(ori_check_time == 0)
//       {
//         time_ori_0 = time;
//         ori_check_time = 1;
//         // std::cout << "check_time" << std::endl;
//       }
//       //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
//       //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

//       //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
//       //target_rotation_ = rotation_f;

//       ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration, tilt_angle_z_, -1.0*tilt_angle_z_, 0, 0);

//       rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

//       target_rotation_ = rotation_z_theta_ * ori_first_state_;
//       //target_rotation_ = ori_first_state_ * rotation_z_theta_; //EE

//       if(time.toSec() > time_ori_0.toSec() + ori_duration)
//       {
//         ori_change_direction = 2;
//         ori_check_time = 0;
//         // std::cout << "check_time_end" << std::endl;
//       }
//     }
//     if(ori_change_direction == 2)
//     {
//       if(ori_check_time == 0)
//       {
//         time_ori_0 = time;
//         ori_check_time = 1;
//         // std::cout << "check_time" << std::endl;
//       }
//       //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
//       //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

//       //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
//       //target_rotation_ = rotation_f;

//       ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration, -1.0*tilt_angle_z_, tilt_angle_z_, 0, 0);

//       rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

//       target_rotation_ = rotation_z_theta_ * ori_first_state_;
//       //target_rotation_ = ori_first_state_ * rotation_z_theta_; //EE

//       if(time.toSec() > time_ori_0.toSec() + ori_duration)
//       {
//         ori_change_direction = 1;
//         ori_check_time = 0;
//         // std::cout << "check_time_end" << std::endl;
//       }
//     }

//     delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);
//     m_star_ = (1.0) * 500 * delphi_delta - 5 * x_dot_.tail<3>(); //100 5

//     f_star_zero_.head(3) = f_star_;
//     f_star_zero_.tail(3) = m_star_;

//     tau_cmd = jacobian.transpose() * (f_star_zero_);

//     /////
//     rotation_z_theta_real_ = ori_first_state_.inverse() * rotation_M;
//     //ori_theta_z_real_ = acos(rotation_z_theta_real_(0));
//     ori_theta_z_real_ = -1*asin(rotation_z_theta_real_(1));
//   }
    else if(pin_state_ == 2)
  {
    if (is_first_)
    {
      spiral_start_time_ = time;
      spiral_origin_ = position;
      // spiral_linear_velocity_ = 0.008; //0.005
      spiral_linear_velocity_ = input_vel_spiral;
      spiral_pitch_ = 0.001; //0.0025 //0.000907
      spiral_duration_ = 3000.0;
      spiral_force_limit_ = 10; //7
      force_press_z_ = -10.0; //-10
      target_rotation_ = rotation_M;
      ori_first_state_ = rotation_M;
      pos_first_state_ = position;
      x_desired_(2) = spiral_origin_(2);
      ori_theta_z_ = 0.0;
      ori_duration = 0.095 * 2;
      tilt_angle_z_ = theta_spiral_; // 10~~~
      if(input_vel_theta != 0)
      {
        ori_duration = (theta_spiral_*180/M_PI/input_vel_theta)*2;
      }
      else
      {
        tilt_angle_z_ = 0.0;
      }

      is_first_ = false;
      std::cout<<"state is 2"<<std::endl;
      // std::cout << "pos_first_state_(2) : " << pos_first_state_(2) << std::endl;
      detect_hole_force = 0;
      std::cout<<"gains: "<<input_p_gain_<<", "<<input_d_gain_<<std::endl;
      std::cout<<"gains 2: "<<input_wp_gain_<<", "<<input_wd_gain_<<std::endl;
      std::cout<<"velocity: "<<input_vel_spiral<<", "<<input_vel_theta<<std::endl;
    }

    spiral_force_ = sqrt(f_measured(0) * f_measured(0) + f_measured(1) * f_measured(1));

    //if (spiral_force_ >= spiral_force_limit_)
    if ((spiral_force_ >= spiral_force_limit_)&&(position(2) < pos_first_state_(2)-0.0008))
    // if ((spiral_force_ >= spiral_force_limit_)&&(position(2) < pos_init_(2)-0.0105))
    //if ((position(2) < pos_first_state_(2)-0.001))
    //if(0)
    {
      pin_state_ = 5; //4 //6
      is_first_ = true;
      ori_check_time = 0;
      ori_change_direction = 0;
      insert_last_z_pos = position(2);
      spiral_time = time - spiral_start_time_;
      std::cout << "SPIRAL MOTIN IS DONE" << std::endl;
      std::cout << "SPIRAL FORCE IS OVER THE LIMIT" << std::endl;
      // std::cout << "pos_first_state_(2) : " << pos_first_state_(2) << std::endl;
      // std::cout << "position(2) : " << position(2) << std::endl;
      std::cout<<"search duration: "<<time.toSec() - spiral_start_time_.toSec()<<std::endl;
      
    }

    if ((spiral_force_ >= spiral_force_limit_)&&(position(2) < pos_first_state_(2)-0.0008))
    {
      detect_hole_force = 1;
      std::cout << "Z - 0.3mm : Hole!!!!!" << std::endl;
    }

    // if (position(2) < pos_first_state_(2)-0.0005)
    // {
    //   //std::cout << "Z - 1mm" << std::endl;
    //   // pin_state_ = 4;
    //   // is_first_ = true;
    //   // ori_check_time = 0;
    //   // ori_change_direction = 0;
    //   // insert_last_z_pos = position(2);
    //   // std::cout << "SPIRAL MOTIN IS DONE" << std::endl;
    // }

    x_desired_.block<2, 1>(0, 0) = DyrosMath::spiral(time.toSec(), spiral_start_time_.toSec(), spiral_start_time_.toSec() + spiral_duration_, spiral_origin_.block<2, 1>(0, 0), spiral_linear_velocity_, spiral_pitch_); //0.0035 //0.02
    x_desired_(2) = spiral_origin_(2);
    xdot_desired_.setZero();                                                                                                                                                                                             // in "approach process", z velocity is not "zero"

    for (int i = 0; i < 3; i++)
    {
      K_p(i, i) = 8000.0; K_v(i, i) = 100.0; //7000
    }

    f_star_ = input_p_gain_ * (x_desired_ - position) + input_d_gain_ * (xdot_desired_ - x_dot_.head<3>());
    f_star_(2) = force_press_z_; //-6, -10
    //f_star_(2) = force_press_z_ + 0.5 * (force_press_z_ - f_sensing(2)) + 100 * (xdot_desired_(2) - x_dot_(2));

///////
    if(ori_change_direction == 0)
    {
      if(ori_check_time == 0)
      {
        time_ori_0 = time;
        ori_check_time = 1;
        std::cout << "check_time" << std::endl;
      }
      //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
      //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

      //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
      //target_rotation_ = rotation_f;

      ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration/2, 0, tilt_angle_z_, 0, 0);

      rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

      target_rotation_ = rotation_z_theta_ * ori_first_state_;
      //target_rotation_ = ori_first_state_ * rotation_z_theta_; //EE

      if(time.toSec() > time_ori_0.toSec() + ori_duration/2)
      {
        ori_change_direction = 1;
        ori_check_time = 0;
        // std::cout << "check_time_end" << std::endl;
      }
    }
    if(ori_change_direction == 1)
    {
      if(ori_check_time == 0)
      {
        time_ori_0 = time;
        ori_check_time = 1;
        // std::cout << "check_time" << std::endl;
      }
      //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
      //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

      //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
      //target_rotation_ = rotation_f;

      ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration, tilt_angle_z_, -1.0*tilt_angle_z_, 0, 0);

      rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

      target_rotation_ = rotation_z_theta_ * ori_first_state_;
      //target_rotation_ = ori_first_state_ * rotation_z_theta_; //EE

      if(time.toSec() > time_ori_0.toSec() + ori_duration)
      {
        ori_change_direction = 2;
        ori_check_time = 0;
        // std::cout << "check_time_end" << std::endl;
      }
    }
    if(ori_change_direction == 2)
    {
      if(ori_check_time == 0)
      {
        time_ori_0 = time;
        ori_check_time = 1;
        // std::cout << "check_time" << std::endl;
      }
      //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
      //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

      //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
      //target_rotation_ = rotation_f;

      ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration, -1.0*tilt_angle_z_, tilt_angle_z_, 0, 0);

      rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

      target_rotation_ = rotation_z_theta_ * ori_first_state_;
      //target_rotation_ = ori_first_state_ * rotation_z_theta_; //EE

      if(time.toSec() > time_ori_0.toSec() + ori_duration)
      {
        ori_change_direction = 1;
        ori_check_time = 0;
        // std::cout << "check_time_end" << std::endl;
      }
    }

    delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);
    // delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, ori_first_state_);

    // m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail<3>(); //100 5
    m_star_ = (1.0) * input_wp_gain_ * delphi_delta - input_wd_gain_ * x_dot_.tail<3>(); //100 5

    m_star_(2) = 0.0;

    f_star_zero_.head(3) = f_star_;
    f_star_zero_.tail(3) = m_star_;

    //tau_cmd = jacobian.transpose() * (f_star_zero_);

    jacobian_6 = jacobian.block(0, 0, 6, 6);
    tau_cmd.head(6) = jacobian_6.transpose() * (f_star_zero_);
    tau_cmd.tail(1).setZero();

    tau_cmd(6) = 50 * ((q_init_(6) + ori_theta_z_) - q(6));

     
    /////
    rotation_z_theta_real_ = ori_first_state_.inverse() * rotation_M;
    //ori_theta_z_real_ = acos(rotation_z_theta_real_(0));
    // ori_theta_z_real_ = -1*asin((-1)*rotation_z_theta_real_(1));
    ori_theta_z_real_ = atan2(rotation_z_theta_real_(1,0),rotation_z_theta_real_(0,0));
    fprintf(gain_tunning, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", x_desired_(0), x_desired_(1), ori_theta_z_, position(0), position(1), ori_theta_z_real_);
  }
  // else if(pin_state_ == 3)
  // {
  //   if(is_first_)
  //   {
  //     rotation_start_time_ = time;
  //     x_desired_ = position;         //to fix x,y position
  //     ori_first_state_ = rotation_M;
  //     ori_return_state_ = rotation_M;
  //     pos_return_state_ = position;
  //     is_first_ = false;
  //     ori_theta_z_ = 0.0;
  //     ori_theta_z_ = 0.0;
  //     force_press_z_ = -30.0;
  //     ori_duration = 1.0;
  //     tilt_angle_z_ = 4*M_PI/180;

  //     std::cout<<"state is 3"<<std::endl;
  //   }

  //   if(time.toSec() > rotation_start_time_.toSec() + 0.5)
  //   {
  //     // pin_state_ = 4;
  //     // is_first_ = true;
  //   }

  //   if(abs(f_sensing(5)) > 6.0)
  //   {
  //     ori_change_direction = 3;
  //   }


  //   f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());

  //   //f_star_zero_.setZero();
  //   f_star_.setZero();
  //   f_star_(2) = force_press_z_; //-6, -10
  //   //f_star_zero_(2) = -20.0;

  //   /////////////////
  //   if(ori_change_direction == 0)
  //   {
  //     if(ori_check_time == 0)
  //     {
  //       time_ori_0 = time;
  //       ori_check_time = 1;
  //       std::cout << "check_time" << std::endl;
  //     }
  //     //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
  //     //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

  //     //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
  //     //target_rotation_ = rotation_f;

  //     ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration/2, 0, tilt_angle_z_, 0, 0);

  //     rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

  //     target_rotation_ = rotation_z_theta_ * ori_first_state_;

  //     if(time.toSec() > time_ori_0.toSec() + ori_duration/2)
  //     {
  //       ori_change_direction = 1;
  //       ori_check_time = 0;
  //       std::cout << "check_time_end" << std::endl;
  //     }
  //   }
  //   if(ori_change_direction == 1)
  //   {
  //     if(ori_check_time == 0)
  //     {
  //       time_ori_0 = time;
  //       ori_check_time = 1;
  //       std::cout << "check_time" << std::endl;
  //     }
  //     //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
  //     //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

  //     //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
  //     //target_rotation_ = rotation_f;

  //     ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration, tilt_angle_z_, -1.0*tilt_angle_z_, 0, 0);

  //     rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

  //     target_rotation_ = rotation_z_theta_ * ori_first_state_;

  //     if(time.toSec() > time_ori_0.toSec() + ori_duration)
  //     {
  //       ori_change_direction = 2;
  //       ori_check_time = 0;
  //       std::cout << "check_time_end" << std::endl;
  //     }
  //   }
  //   if(ori_change_direction == 2)
  //   {
  //     if(ori_check_time == 0)
  //     {
  //       time_ori_0 = time;
  //       ori_check_time = 1;
  //       std::cout << "check_time" << std::endl;
  //     }
  //     //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
  //     //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

  //     //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
  //     //target_rotation_ = rotation_f;

  //     ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration, -1.0*tilt_angle_z_, tilt_angle_z_, 0, 0);

  //     rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

  //     target_rotation_ = rotation_z_theta_ * ori_first_state_;

  //     if(time.toSec() > time_ori_0.toSec() + ori_duration)
  //     {
  //       ori_change_direction = 1;
  //       ori_check_time = 0;
  //       std::cout << "check_time_end" << std::endl;
  //     }
  //   }

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);
  //   m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail(3);//100 5

  //   m_star_.setZero();

  //   f_star_zero_.head(3) = f_star_;
  //   f_star_zero_.tail(3) = m_star_;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);

  //   // rotation_z_theta_real_ = ori_first_state_.inverse() * rotation_M;
  //   // ori_theta_z_real_ = acos(rotation_z_theta_real_(0));
  // }

  // else if(pin_state_ == 4)
  // {
  //   if(is_first_)
  //   {
  //     rotation_start_time_ = time;
  //     x_desired_ = position;         //to fix x,y position
  //     ori_first_state_ = rotation_M;
  //     ori_return_state_ = rotation_M;
  //     pos_return_state_ = position;
  //     is_first_ = false;
  //     ori_theta_z_ = 0.0;
  //     force_press_z_ = -30.0;
  //     ori_duration = 1.0;
  //     tilt_angle_z_ = 2*M_PI/180;
  //     tilt_angle_z_last_ = 0.0;

  //     std::cout<<"state is 3"<<std::endl;
  //   }

  //   if(time.toSec() > rotation_start_time_.toSec() + 0.5)
  //   {
  //     // pin_state_ = 4;
  //     // is_first_ = true;
  //   }

  //   if(abs(f_sensing(5)) > 6.0)
  //   {
  //     ori_change_direction = 3;
  //   }


  //   f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());

  //   //f_star_zero_.setZero();
  //   f_star_.setZero();
  //   f_star_(2) = force_press_z_; //-6, -10
  //   //f_star_zero_(2) = -20.0;

  //   /////////////////
  //   if(ori_change_direction == 0)
  //   {
  //     if(ori_check_time == 0)
  //     {
  //       time_ori_0 = time;
  //       ori_check_time = 1;
  //       std::cout << "check_time" << std::endl;
  //       insert_last_z_pos = position(2);
  //     }
  //     //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
  //     //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

  //     //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
  //     //target_rotation_ = rotation_f;

  //     ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration/2, tilt_angle_z_last_, tilt_angle_z_, 0, 0);

  //     rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

  //     target_rotation_ = rotation_z_theta_ * ori_first_state_;

  //     if(time.toSec() > time_ori_0.toSec() + ori_duration/2)
  //     {
  //       //ori_change_direction = 0;
  //       ori_check_time = 0;
  //       std::cout << "check_time_end" << std::endl;

  //       tilt_angle_z_last_ = tilt_angle_z_;

  //       if(insert_last_z_pos > position(2))
  //       {
  //         ori_change_direction = 2;
  //         std::cout << "2: insert_last_z_pos" << insert_last_z_pos << std::endl;
  //         std::cout << "position(2)" << position(2) << std::endl;

  //       }
  //       if(insert_last_z_pos < position(2))
  //       {
  //         ori_change_direction = 1;
  //         std::cout << "1: insert_last_z_pos" << insert_last_z_pos << std::endl;
  //         std::cout << "position(2)" << position(2) << std::endl;
  //       }
  //     }
  //   }
  //   if(ori_change_direction == 1)
  //   {
  //     if(ori_check_time == 0)
  //     {
  //       time_ori_0 = time;
  //       ori_check_time = 1;
  //       std::cout << "check_time" << std::endl;
  //     }
  //     //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
  //     //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

  //     //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
  //     //target_rotation_ = rotation_f;

  //     ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration, tilt_angle_z_, -2.0*tilt_angle_z_, 0, 0);

  //     rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

  //     target_rotation_ = rotation_z_theta_ * ori_first_state_;

  //     if(time.toSec() > time_ori_0.toSec() + ori_duration)
  //     {
  //       //ori_change_direction = 2;
  //       //ori_check_time = 0;
  //       //std::cout << "check_time_end" << std::endl;
  //     }
  //   }
  //   if(ori_change_direction == 2)
  //   {
  //     if(ori_check_time == 0)
  //     {
  //       time_ori_0 = time;
  //       ori_check_time = 1;
  //       std::cout << "check_time" << std::endl;
  //     }
  //     //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
  //     //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

  //     //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
  //     //target_rotation_ = rotation_f;

  //     ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration/2, tilt_angle_z_, 2*tilt_angle_z_, 0, 0);

  //     rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

  //     target_rotation_ = rotation_z_theta_ * ori_first_state_;

  //     if(time.toSec() > time_ori_0.toSec() + ori_duration/2)
  //     {
  //       //ori_change_direction = 1;
  //       //ori_check_time = 0;
  //       //std::cout << "check_time_end" << std::endl;
  //     }
  //   }

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);
  //   m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail(3);//100 5

  //   f_star_zero_.head(3) = f_star_;
  //   f_star_zero_.tail(3) = m_star_;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);

  //   // rotation_z_theta_real_ = ori_first_state_.inverse() * rotation_M;
  //   // ori_theta_z_real_ = acos(rotation_z_theta_real_(0));
  // }


  ///////////////////
  else if(pin_state_ == 5) //align z-angle!!!
  {
    if(is_first_)
    {
      rotation_start_time_ = time;
      x_desired_ = position;         //to fix x,y position
      ori_first_state_ = rotation_M;
      ori_return_state_ = rotation_M;
      pos_return_state_ = position;
      //pos_first_state_ = position;
      is_first_ = false;
      ori_theta_z_ = 0.0;
      force_press_z_ = -30.0;
      ori_duration = 2.5; //1.5 //0.9
      // ori_duration = (5/input_vel_theta)*2;
      tilt_angle_z_ = 5*M_PI/180;

      ori_check_time = 0;
      ori_change_direction = 0;

      std::cout<<"state is 5"<<std::endl;

      // if(f_sensing_z_avr >=0)
      // {
      //   ori_change_direction = 1;
      // }
      // if(f_sensing_z_avr <0)
      // {
      //   ori_change_direction = 2;
      // }
    }

    // if(ori_change_direction < 3)
    // {
    //   if((position(2) < pos_first_state_(2)-0.005)||(abs(f_measured(5)) > 2.0))
    //   {
    //     ori_change_direction = 4;
    //     std::cout<<"ori_change_direction : "<<ori_change_direction<<std::endl;
    //     // spiral_start_time_ = time;
    //   }
    // }

    if ((position(2) < pos_first_state_(2)-0.007))//&&(ori_change_direction != 3))
    {
      std::cout << "Z - 8mm" << std::endl;
      ori_change_direction = 3;
      finish_time = simulation_time;
      insert_time = time - rotation_start_time_;
      pin_state_ = 8;
      is_first_ = true;

      fprintf(save_data_x3, "%lf\t %lf\t %lf\t %lf\t\n", finish_time.toSec(), approach_time.toSec(), spiral_time.toSec(), insert_time.toSec());
    }


    f_star_ = 500 * (x_desired_ - position) + 20 * (xdot_desired_ - x_dot_.head<3>());

    //f_star_zero_.setZero();
    f_star_.setZero();
    f_star_(2) = force_press_z_; //-6, -10
    //f_star_zero_(2) = -20.0;

    /////////////////
    if(ori_change_direction == 0)
    {
      if(ori_check_time == 0)
      {
        time_ori_0 = time;
        ori_check_time = 1;
        std::cout<<"ori_change_direction : "<<ori_change_direction<<std::endl;
        // std::cout << "check_time" << std::endl;
      }
      //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
      //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

      //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
      //target_rotation_ = rotation_f;

      ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration/2, 0, tilt_angle_z_/2, 0, 0);

      rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

      target_rotation_ = rotation_z_theta_ * ori_first_state_;

      if(time.toSec() > time_ori_0.toSec() + ori_duration/2)
      {
        ori_change_direction = 1;
        ori_check_time = 0;
        // std::cout<<"ori_change_direction : "<<ori_change_direction<<std::endl;
        // std::cout << "check_time_end" << std::endl;
      }
    }
    if(ori_change_direction == 1)
    {
      if(ori_check_time == 0)
      {
        time_ori_0 = time;
        ori_check_time = 1;
        ori_theta_z_ = 0.0;
        std::cout<<"ori_change_direction : "<<ori_change_direction<<std::endl;
        // std::cout << "check_time" << std::endl;
      }
      //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
      //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

      //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
      //target_rotation_ = rotation_f;

      ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration, 0.0, -1.0*tilt_angle_z_, 0, 0);

      rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

      target_rotation_ = rotation_z_theta_ * ori_first_state_;

      if(time.toSec() > time_ori_0.toSec() + ori_duration)
      {
        ori_change_direction = 2;
        ori_check_time = 0;
        //std::cout << "check_time_end" << std::endl;
      }
    }
    if(ori_change_direction == 2)
    {
      if(ori_check_time == 0)
      {
        time_ori_0 = time;
        ori_check_time = 1;
        ori_theta_z_ = 0.0;
        std::cout<<"ori_change_direction : "<<ori_change_direction<<std::endl;
        // std::cout << "check_time" << std::endl;
      }
      //rotation_0 << 0, sin(0), cos(0), 0, cos(0), -sin(0), -1, 0, 0;
      //rotation_f << 0, sin(5*M_PI/180), cos(5*M_PI/180), 0, cos(5*M_PI/180), -sin(5*M_PI/180), -1, 0, 0;

      //target_rotation_ = DyrosMath::rotationCubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + 10.0, rotation_0, rotation_f);
      //target_rotation_ = rotation_f;

      ori_theta_z_ = DyrosMath::cubic(time.toSec(), time_ori_0.toSec(), time_ori_0.toSec() + ori_duration, 0.0, tilt_angle_z_, 0, 0);

      rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;

      target_rotation_ = rotation_z_theta_ * ori_first_state_; //global

      if(time.toSec() > time_ori_0.toSec() + ori_duration)
      {
        ori_change_direction = 1;
        ori_check_time = 0;
        //std::cout << "check_time_end" << std::endl;
      }
    }

    K_p_ori.setZero();
    K_p_ori(0, 0) = 200.0; //300
    K_p_ori(1, 1) = 200.0; //300
    K_p_ori(2, 2) = 200.0; //250

    delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

    // if(ori_change_direction < 3)
    // {
      m_star_ = (0.5) * K_p_ori * delphi_delta - 0.5 * x_dot_.tail(3);//100 5
      // m_star_(2) = 0.0;
    // }
    // else
    // {
    //   m_star_.setZero();
    // }

    // m_star_.setZero();
    // m_star_.setZero();

    // if ((ori_change_direction == 4)&&(time.toSec() < spiral_start_time_.toSec() + 1.0))
    // {
    //   if(f_sensing_z_avr >=0)
    //   {
    //     m_star_(2) = 1.0;

    //     std::cout << "m_star_(2) = 1.0" << std::endl;
    //   }
    //   if(f_sensing_z_avr <0)
    //   {
    //     m_star_(2) = -1.0;
    //     std::cout << "m_star_(2) = -1.0" << std::endl;
    //   }
    // }
    // m_star_.setZero();
    f_star_zero_.head(3) = f_star_;
    f_star_zero_.tail(3) = m_star_;

    tau_cmd = jacobian.transpose() * (f_star_zero_);
  }

  else if(pin_state_ == 6)
  {
    if(is_first_)
    {
      rotation_start_time_ = time;
      x_desired_ = position;
      ori_first_state_ = rotation_M;
      is_first_ = false;
      ori_theta_z_ = 0.0;

      f_sensing_z_sum = 0.0;
      f_sensing_z_avr = 0.0;
      f_sensing_z_num = 0.0;
      f_sensing_x_sum = 0.0;
      f_sensing_x_avr = 0.0;
      f_sensing_x_num = 0.0;
      f_sensing_y_sum = 0.0;
      f_sensing_y_avr = 0.0;
      f_sensing_y_num = 0.0;

      std::cout<<"state is 6"<<std::endl;
    }

    if(time.toSec() > rotation_start_time_.toSec() + 0.1)
    {
      pin_state_ = 5;
      is_first_ = true;
      //x_last_desired_2_ = position;

      f_sensing_z_avr = f_sensing_z_sum / f_sensing_z_num;
      f_sensing_x_avr = f_sensing_x_sum / f_sensing_x_num;
      f_sensing_y_avr = f_sensing_y_sum / f_sensing_y_num;
      std::cout<<"f_sensing_z_avr : "<< f_sensing_z_avr <<std::endl;
      // std::cout<<"f_sensing_x_avr : "<< f_sensing_x_avr <<std::endl;
      // std::cout<<"f_sensing_y_avr : "<< f_sensing_y_avr <<std::endl;
    }


    if(time.toSec() > rotation_start_time_.toSec() + 0.05)
    {
      f_sensing_z_sum = f_sensing_z_sum + f_measured(5);
      f_sensing_z_num = f_sensing_z_num + 1.0;
      f_sensing_x_sum = f_sensing_x_sum + f_measured(3);
      f_sensing_x_num = f_sensing_x_num + 1.0;
      f_sensing_y_sum = f_sensing_y_sum + f_measured(4);
      f_sensing_y_num = f_sensing_y_num + 1.0;
    }

    f_star_ = 400 * (x_desired_ - position) + 40 * (xdot_desired_ - x_dot_.head(3));

    delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, ori_first_state_);
    m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail(3);//100 5


    // m_star_.setZero();
    f_star_.setZero();


    f_star_zero_.head(3) = f_star_;
    f_star_zero_.tail(3) = m_star_;

    //f_star_zero_.setZero();
    //f_star_zero_(1) = 15.0;
    f_star_zero_(2) = -15.0;
    //f_star_zero_(3) = 0.0;
    //f_star_zero_(4) = 0.0;
    // f_star_zero_(5) = 0.0;
    //f_star_zero_(3) = 2.0;
    //f_star_zero_(4) = -2.0;

    tau_cmd = jacobian.transpose() * (f_star_zero_);
  }
  // else if(pin_state_ == 7)
  // {
  //   if(is_first_)
  //   {
  //     rotation_start_time_ = time;
  //     strategy_start_time_ = time;
  //     x_desired_ = position;
  //     ori_first_state_ = rotation_M;
  //     is_first_ = false;
  //     ori_theta_z_ = 0.0;
  //     std::cout<<"state is 7"<<std::endl;

  //   }

  //   if(time.toSec() > rotation_start_time_.toSec() + 2)
  //   {

  //     pin_state_ = 6;
  //     is_first_ = true;
  //   }


  //   f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
  //   m_star_ = keepOrientationPerpenticularOnlyXY(ori_first_state_, rotation_M, x_dot_, 0.1, time.toSec(), strategy_start_time_.toSec());

  //   f_star_zero_.head(3) = f_star_;
  //   f_star_zero_.tail(3) = m_star_;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);
  //   //tau_cmd.setZero();

  //   // fprintf(save_data_x3, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_ee(0), f_ee(1), f_ee(2), m_ee(0), m_ee(1), m_ee(2));
  // }
  else if(pin_state_ == 8)
  {
    if(is_first_)
    {
      rotation_start_time_ = time;
      strategy_start_time_ = time;
      x_desired_ = position;
      //x_desired_(2) = pos_first_state_(2)-0.010;
      ori_first_state_ = rotation_M;
      is_first_ = false;
      ori_theta_z_ = 0.0;
      std::cout<<"state is 8"<<std::endl;
      m_star_.setZero();

    }
    int dt = time.toSec() - strategy_start_time_.toSec();


    // delphi_delta = -0.5 * DyrosMath::getPhi(ori_first_state_, ori_init_);

    // m_star_ = (1.0) * K_p_ori * delphi_delta - 5 * x_dot_.tail(3);//100 5

//    m_star_.setZero();

    // if(dt % 2 == 0)
    // {
    //   m_star_(0) = -1.0;//-f_sensing_x_avr*3;
    //   m_star_(1) = -1.0;//-f_sensing_y_avr*3;
    //   m_star_(2) = -1.0;//-f_sensing_y_avr*3;

    // }
    // else
    // {
    //   m_star_(0) = 1.0;//f_sensing_x_avr*3;
    //   m_star_(1) = 1.0;//_sensing_y_avr*3;
    //   m_star_(2) = 1.0;//_sensing_y_avr*3;
    // }


    K_p(0,0) = 0;
    K_p(1,1) = 0;

    f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
    f_star_(0) = 0.0;
    f_star_(1) = 0.0;
    f_star_(2) = -25.0;
    //m_star_ = keepOrientationPerpenticularOnlyXY(ori_first_state_, rotation_M, x_dot_, 0.1, time.toSec(), strategy_start_time_.toSec());

    m_star_.setZero();
    f_star_zero_.head(3) = f_star_;
    f_star_zero_.tail(3) = m_star_;

    // std::cout<<"command: "<<f_star_(2)<<" "<<m_star_.transpose()<<std::endl;
    tau_cmd = jacobian.transpose() * (f_star_zero_);

    if ((position(2) < pos_first_state_(2)-0.014))
    {
        is_done_ = true;
        pin_state_ = 9;
        std::cout << "Z - 14mm" << std::endl;
        f_star_.setZero();
        m_star_.setZero();
    }
    //tau_cmd.setZero();
  }

  else if(pin_state_ == 9)
  {
    if(is_done_)
    {
      std::cout << "Z - 14mm" << std::endl;
      is_done_ = false;
    }
    tau_cmd.setZero();
  }
  // else if(pin_state_ == 20)
  // {
  //   if(is_first_)
  //   {
  //     rotation_start_time_ = time;
  //     //x_desired_ = pos_init_;
  //     spiral_origin_ = position;
  //     ori_first_state_ = rotation_M;
  //     is_first_ = false;

  //     std::cout<<"state is 20"<<std::endl;
  //   }

  //   if(time.toSec() > rotation_start_time_.toSec() + 6.0)
  //   {
  //     //pin_state_ = 0;
  //     //is_first_ = true;
  //   }

  //   for(int i=0;i<2;i++)
  //   {
  //     x_desired_(i) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + 5.0, spiral_origin_(i), pos_init_(i), 0, 0);
  //   }

  //   target_rotation_ = DyrosMath::rotationCubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + 5.0, ori_init_, ori_first_state_);

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

  //   f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
  //   m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail<3>(); //100 5

  //   f_star_zero_.head(3) = f_star_;
  //   f_star_zero_.tail(3) = m_star_;

  //   f_star_zero_(2) = 3.0;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);
  // }

  // else if(pin_state_ == 31)
  // {
  //   if(is_first_)
  //   {
  //     rotation_start_time_ = time;
  //     recovery_start_time_ = time;
  //     x_desired_ = position;
  //     ori_first_state_ = rotation_M;
  //     is_first_ = false;
  //     ori_theta_z_ = 0.0;

  //     rotation_duration_ = 1.0;
  //     tilt_angle_z_ = 10*M_PI/180;

  //     //rotation_z_direction_ = true;

  //     std::cout<<"state is 31"<<std::endl;
  //   }

  //   //if(ori_theta_z_ >= tilt_angle_z_)
  //   // if(abs(f_sensing(5)) >= 1.5)
  //   // {
  //   //   //check_orientation_ = true;
  //   //   pin_state_ = 5; // check_orientation -> contact
  //   //   is_first_ = true;
  //   //   std::cout<<"ORIENTATION IS DONE"<<std::endl;
  //   // }

  //   if(time.toSec() > rotation_start_time_.toSec() + rotation_duration_ + 0.01)
  //   {
  //     //rotation_z_direction_ = false;
  //     is_first_ = true;
  //     pin_state_ = 32;
  //   }

  //   // moment_xy = sqrt(f_sensing(3) * f_sensing(3) + f_sensing(4) * f_sensing(4));
  //   // std::cout<<"moment_xy: "<< moment_xy <<std::endl;

  //   //x_desired_ = pos_init_;

  //   ori_theta_z_ = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + rotation_duration_, 0, tilt_angle_z_, 0, 0);
  //   //rotation_z_theta_ << cos(-ori_theta_z_), -sin(-ori_theta_z_), 0, sin(-ori_theta_z_), cos(-ori_theta_z_), 0, 0, 0, 1;
  //   //rotation_z_theta_ << 1, 0, 0, 0, cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_); //x
  //   rotation_z_theta_ << cos(-ori_theta_z_), 0, sin(-ori_theta_z_), 0, 1, 0, -sin(-ori_theta_z_), 0, cos(-ori_theta_z_); //y
  //   target_rotation_ = ori_first_state_*rotation_z_theta_;

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

  //   f_star_ = 400 * (x_desired_ - position) + 40 * (xdot_desired_ - x_dot_.head(3));
  //   m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail(3);//100 5

  //   f_star_zero_.head(3) = f_star_;
  //   //f_star_zero_(2) = -10.0;
  //   f_star_zero_.setZero();
  //   f_star_zero_.tail(3) = m_star_;
  //   //f_star_zero_(3) = 0.0;
  //   //f_star_zero_(4) = 0.0;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);
  // }
  // else if(pin_state_ == 32)
  // {
  //   if(is_first_)
  //   {
  //     rotation_start_time_ = time;
  //     x_desired_ = position;
  //     ori_first_state_ = rotation_M;
  //     is_first_ = false;
  //     ori_theta_z_ = 0.0;

  //     rotation_duration_ = 1.0;
  //     tilt_angle_z_ = -5*M_PI/180;

  //     //rotation_z_direction_ = true;

  //     std::cout<<"state is 32"<<std::endl;
  //   }

  //   //if(ori_theta_z_ >= tilt_angle_z_)
  //   // if(abs(f_sensing(5)) >= 1.5)
  //   // {
  //   //   //check_orientation_ = true;
  //   //   pin_state_ = 5; // check_orientation -> contact
  //   //   is_first_ = true;
  //   //   std::cout<<"ORIENTATION IS DONE"<<std::endl;
  //   // }

  //   if(time.toSec() > rotation_start_time_.toSec() + rotation_duration_ + 0.01)
  //   {
  //     //rotation_z_direction_ = false;
  //     is_first_ = true;
  //     pin_state_ = 3;
  //   }

  //   // moment_xy = sqrt(f_sensing(3) * f_sensing(3) + f_sensing(4) * f_sensing(4));
  //   // std::cout<<"moment_xy: "<< moment_xy <<std::endl;

  //   //x_desired_ = pos_init_;

  //   ori_theta_z_ = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + rotation_duration_, 0, tilt_angle_z_, 0, 0);
  //   rotation_z_theta_ << cos(-ori_theta_z_), -sin(-ori_theta_z_), 0, sin(-ori_theta_z_), cos(-ori_theta_z_), 0, 0, 0, 1;
  //   //rotation_z_theta_ << 1, 0, 0, 0, cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_); //x
  //   //rotation_z_theta_ << cos(-ori_theta_z_), 0, sin(-ori_theta_z_), 0, 1, 0, -sin(-ori_theta_z_), 0, cos(-ori_theta_z_); //y
  //   target_rotation_ = ori_first_state_*rotation_z_theta_;

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

  //   f_star_ = 400 * (x_desired_ - position) + 40 * (xdot_desired_ - x_dot_.head(3));
  //   m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail(3);//100 5

  //   f_star_zero_.head(3) = f_star_;
  //   //f_star_zero_(2) = -10.0;
  //   f_star_zero_.setZero();
  //   f_star_zero_.tail(3) = m_star_;
  //   //f_star_zero_(3) = 0.0;
  //   //f_star_zero_(4) = 0.0;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);

  //   // Eigen::Vector3d euler_angle_state_ = DyrosMath::rot2Euler(ori_return_state_.inverse() * rotation_M);
  //   // if(print_rate_trigger_()){
  //   // ROS_INFO_STREAM("euler_angle_state_ : "<< euler_angle_state_.transpose());
  //   // }
  // }

  // else if(pin_state_ == 34)
  // {
  //   if(is_first_)
  //   {
  //     rotation_start_time_ = time;
  //     pos_first_state_ = position;
  //     ori_first_state_ = rotation_M;
  //     is_first_ = false;
  //     ori_theta_z_ = 0.0;

  //     rotation_duration_ = 5.0;
  //     tilt_angle_z_ = -10*M_PI/180;

  //     rotation_z_theta_ << cos(5*M_PI/180), 0, sin(5*M_PI/180), 0, 1, 0, -sin(5*M_PI/180), 0, cos(5*M_PI/180);
  //     ori_return_state_ = ori_return_state_ * rotation_z_theta_;
  //     pos_return_state_(2) = pos_return_state_(2) + 0.01;

  //     //rotation_z_direction_ = true;

  //     std::cout<<"state is 34"<<std::endl;
  //   }

  //   //if(ori_theta_z_ >= tilt_angle_z_)
  //   // if(abs(f_sensing(5)) >= 1.5)
  //   // {
  //   //   //check_orientation_ = true;
  //   //   pin_state_ = 5; // check_orientation -> contact
  //   //   is_first_ = true;
  //   //   std::cout<<"ORIENTATION IS DONE"<<std::endl;
  //   // }

  //   // if(time.toSec() > rotation_start_time_.toSec() + rotation_duration_ + 1.0)
  //   // {
  //   //   rotation_z_direction_ = false;
  //   //   is_first_ = true;
  //   //   pin_state_ = 20;
  //   // }

  //   // moment_xy = sqrt(f_sensing(3) * f_sensing(3) + f_sensing(4) * f_sensing(4));
  //   // std::cout<<"moment_xy: "<< moment_xy <<std::endl;

  //   //x_desired_ = pos_init_;

  //   //ori_theta_z_ = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + rotation_duration_, 0, tilt_angle_z_, 0, 0);
  //   //rotation_z_theta_ << cos(-ori_theta_z_), -sin(-ori_theta_z_), 0, sin(-ori_theta_z_), cos(-ori_theta_z_), 0, 0, 0, 1;
  //   //rotation_z_theta_ << 1, 0, 0, 0, cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_); //x
  //   //rotation_z_theta_ << cos(-ori_theta_z_), 0, sin(-ori_theta_z_), 0, 1, 0, -sin(-ori_theta_z_), 0, cos(-ori_theta_z_); //y
  //   //target_rotation_ = ori_first_state_*rotation_z_theta_;

  //   for(int i=0;i<2;i++)
  //   {
  //     x_desired_(i) = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + rotation_duration_, pos_first_state_(i), pos_return_state_(i), 0, 0);
  //   }

  //   target_rotation_ = DyrosMath::rotationCubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + rotation_duration_, ori_first_state_, ori_return_state_);

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

  //   f_star_ = 2000 * (x_desired_ - position) + 50 * (xdot_desired_ - x_dot_.head(3));
  //   m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail(3);//100 5

  //   f_star_zero_.head(3) = f_star_;
  //   //f_star_zero_(2) = -10.0;
  //   //f_star_zero_.setZero();
  //   f_star_zero_.tail(3) = m_star_;
  //   //f_star_zero_(3) = 0.0;
  //   //f_star_zero_(4) = 0.0;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);
  // }


  // if(check_orientation_ == false) //Do rotate E.E
  // {
  //   if(is_check_orientation_first_)
  //   {
  //     rotation_start_time_ = time;
  //     is_check_orientation_first_ = false;
  //   }

  //   if(ori_theta_z_ >= tilt_angle_z_)
  //   {
  //     check_orientation_ = true;
  //     std::cout<<"ORIENTATION IS DONE"<<std::endl;
  //   }

  //   x_desired_ = pos_init_;

  //   ori_theta_z_ = DyrosMath::cubic(time.toSec(), rotation_start_time_.toSec(), rotation_start_time_.toSec() + rotation_duration_, 0, tilt_angle_z_, 0, 0);
  //   //rotation_z_theta_ << cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_), 0, 0, 0, 1;
  //   //rotation_z_theta_ << 1, 0, 0, 0, cos(ori_theta_z_), -sin(ori_theta_z_), 0, sin(ori_theta_z_), cos(ori_theta_z_); //x
  //   rotation_z_theta_ << cos(-ori_theta_z_), 0, sin(-ori_theta_z_), 0, 1, 0, -sin(-ori_theta_z_), 0, cos(-ori_theta_z_); //y
  //   target_rotation_ = ori_init_*rotation_z_theta_;

  //   delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

  //   f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head(3));
  //   m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail(3);//100 5

  //   f_star_zero_.head(3) = f_star_;
  //   f_star_zero_.tail(3) = m_star_;

  //   tau_cmd = jacobian.transpose() * (f_star_zero_);
  // }

  // else{
  //   if(check_contact == false) //Do approace E.E toward the graound
  //     {
  //       if(is_check_contact_first_)
  //       {
  //         target_rotation_ = rotation_M;  //to keep the initial orientation
  //         x_desired_ = position; //to fix x,y position
  //         xdot_desired_(2) = descent_speed_;
  //         contact_force_ = -0.05;
  //         is_check_contact_first_ = false;

  //       }

  //       if(f_sensing(2) <= contact_force_) //f_z is changed frome positive value to negative.
  //       {
  //         check_contact = true; // after finishing the whole process, this value must be changed to "false"
  //         //pos_init_(2) = position(2);
  //         tau_cmd = jacobian.transpose() * (f_star_zero_.setZero());
  //         std::cout<<"CONTACT IS DETECTED"<<std::endl;
  //       }

  //       delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

  //       f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
  //       m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail<3>();//100 5

  //       f_star_zero_.head(3) = f_star_;
  //       f_star_zero_.tail(3) = m_star_;

  //       tau_cmd = jacobian.transpose() * (f_star_zero_);

  //       x_desired_(2) += descent_speed_ / 1000.0;

  //     }

  //   else{
  //     if(check_spiral_done_ == false) //Do spiral motion
  //     {
  //       if(is_spiral_motion_first_)
  //       {
  //         spiral_start_time_ = time;
  //         spiral_origin_ = position;
  //         spiral_linear_velocity_ = 0.005;
  //         spiral_pitch_ = 0.002;
  //         spiral_duration_ = 90.0;
  //         spiral_force_limit_ = 5;
  //         target_rotation_ = rotation_M;
  //         x_desired_(2) = spiral_origin_(2);
  //         is_spiral_motion_first_ = false;

  //       }

  //       spiral_force_ = sqrt(f_sensing(0)*f_sensing(0) + f_sensing(1)*f_sensing(1));

  //       if(spiral_force_ >= spiral_force_limit_)
  //       {
  //         check_spiral_done_ = true;
  //         tau_cmd = jacobian.transpose() * (f_star_zero_.setZero());
  //         std::cout<<"SPIRAL MOTIN IS DONE"<<std::endl;
  //       }

  //       x_desired_.block<2, 1>(0, 0) = DyrosMath::spiral(time.toSec(), spiral_start_time_.toSec(), spiral_start_time_.toSec() + spiral_duration_, spiral_origin_.block<2, 1>(0, 0), spiral_linear_velocity_, spiral_pitch_); //0.0035 //0.02
  //       xdot_desired_.setZero(); // in "approach process", z velocity is not "zero"

  //       delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

  //       f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
  //       //f_star_(2) = -1.0;
  //       m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail<3>();//100 5

  //       f_star_zero_.head(3) = f_star_;
  //       f_star_zero_.tail(3) = m_star_;

  //       tau_cmd = jacobian.transpose() * (f_star_zero_);
  //     }

  //     else
  //     {
  //       if(check_curved_approach_ == false) // Do curved approach
  //       {
  //         if(is_curved_approach_first_)
  //         {
  //           target_rotation_ = rotation_M;
  //           curved_approach_distance_ = 0.2687; // pin to pin distance
  //           curved_approach_angle_ = 0.1*M_PI/180; //
  //           curved_approach_lin_vel_ = 0; //CHANGE IT LATER!!!!!!!
  //           x_desired_ = position;
  //           is_curved_approach_first_ = false;
  //         }

  //         curved_approach_force_ = sqrt(f_sensing(0)*f_sensing(0) + f_sensing(1)*f_sensing(1));

  //         if(curved_approach_force_ <= -1.0)
  //         {
  //           check_curved_approach_ = true;
  //           tau_cmd = jacobian.transpose() * (f_star_zero_.setZero());
  //           std::cout<<"CURVED APPROCH IS DONE"<<std::endl;

  //         }

  //         delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_);

  //         f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head<3>());
  //         m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail<3>();//100 5

  //         f_star_zero_.head(3) = f_star_;
  //         f_star_zero_.tail(3) = m_star_;

  //         tau_cmd = jacobian.transpose() * (f_star_zero_);

  //         x_desired_(1) = position(1) + curved_approach_distance_*cos(-curved_approach_angle_);
  //         x_desired_(2) = position(2) + curved_approach_distance_*sin(-curved_approach_angle_);

  //       }

  //       else
  //       {
  //         if(check_yaw_motion_ == false)
  //         {

  //         }


  //       }
  //     }


//////////////////////////////////////////////////////////////////////////////////////
        /*if ((position(2) < pos_init_(2)-0.003)&&(check_stop == 0))
        {
          check_stop = 1;

          stop_x = position;
          finish_time = simulation_time;

        }



          if(check_stop == 1)
          {
            x_desired_ = stop_x;
          }


          K_p.setZero(); K_v.setZero();
          for (int i = 0; i < 3; i++)
          {
            K_p(i, i) = 5000.0; K_v(i, i) = 100.0; //7000
          }
          K_p(2, 2) = 3000.0; //5000

          f_star_ = K_p * (x_desired_ - position) + K_v * (xdot_desired_ - x_dot_.head(3));

          //f_star_(0) = 0.0;
          //f_star_(1) = 0.0;
          //f_star_(2) = -6.0; // put some value!!! //-6 //-12

          if(check_stop == 1)
          {
            //f_star_(2) = -60.0;//-20
          }


        //  target_rotation_ << 0, sin(ori_theta), cos(ori_theta), 0, cos(ori_theta), -sin(ori_theta), -1, 0, 0;

          delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, ori_init_);


          // K_p_ori.setZero();
          // for (int i = 0; i < 3; i++)
          // {
          // 	K_p_ori(i, i) = 100.0; //100
          // }

          // K_p_ori(0,0) = 0.0;

          m_star_ = (1.0) * 200 * delphi_delta - 5 * x_dot_.tail(3);//100 5

          //m_star_.setZero();

          f_star_zero_.head(3) = f_star_;
          f_star_zero_.tail(3) = m_star_;

          tau_cmd = jacobian.transpose() * (f_star_zero_); */

          //tau_cmd = jacobian.transpose() * (f_star_zero_ + target_f);


          /*if (sqrt(f_sensing(0)*f_sensing(0)+f_sensing(1)*f_sensing(1))>=21&&(check_stop == 0))
          {
            check_stop = 1;

            stop_x = position;
            finish_time = simulation_time;

          }*/

          ////////////////////



        //   if(!check_contact)
        // {
        //   f_star_zero_.setZero();
        //   f_star_zero_(2) = -4;
        //   tau_cmd = jacobian.transpose() * (f_star_zero_);
        // }



        // copy and paste until here!!!
        //////////////////////////////////////////////////////////

          // double kp, kv;
          // kp = 1000;
          // kv = 40;

          // Eigen::Matrix<double , 7, 7> A_diag;
          // A_diag.setIdentity();

          // for(int i=0; i<7; i++){
          //   A_diag(i, i) = mass_matrix(i, i);
          // }

          //tau_cmd =  mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd));// + coriolis;

          //tau_cmd.setZero();

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
    // ROS_INFO("--------------------------------------------------");
    //ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    // ROS_INFO_STREAM("time :"<< simulation_time);
    //ROS_INFO_STREAM("x_curent : "<< position.transpose());
    //ROS_INFO_STREAM("x_desired : "<< x_desired_.transpose());
    //ROS_INFO_STREAM("mass :" << mass_matrix);
    // ROS_INFO_STREAM("ori_theta_z :"<< ori_theta_z_*180/M_PI);
    // ROS_INFO_STREAM("f_sensing : "<< f_sensing.transpose());
    // ROS_INFO_STREAM("spiral_force_ : "<< spiral_force_);
    // ROS_INFO_STREAM("finish_time : "<< finish_time);
    // ROS_INFO_STREAM("approach_time : "<< approach_time);
    // ROS_INFO_STREAM("spiral_time : "<< spiral_time);
    // ROS_INFO_STREAM("insert_time : "<< insert_time);
    // ROS_INFO_STREAM("delphi_delta : "<< delphi_delta.transpose()*180/M_PI);

    // ROS_INFO_STREAM("pin_state_ : "<< pin_state_);
    // ROS_INFO_STREAM("exp_num : "<< exp_num);
    // ROS_INFO_STREAM("last_z_pos_avr_2 : "<< last_z_pos_avr_2);
    // ROS_INFO_STREAM("detect_hole_force : "<< detect_hole_force);

    // ROS_INFO_STREAM("euler_angle : "<< euler_angle.transpose());

  }

  fprintf(joint0_data, "%lf\t %lf\t\n", ori_theta_z_, ori_theta_z_real_);
  fprintf(save_data_x2, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", position(0), position(1), position(2), x_desired_(0), x_desired_(1), x_desired_(2), euler_angle(0), euler_angle(1), euler_angle(2), spiral_force_);
  fprintf(save_data_x, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", tau_cmd(0), tau_cmd(1), tau_cmd(2), tau_cmd(3), tau_cmd(4), tau_cmd(5), tau_cmd(6));
  //fprintf(save_data_x2, "%lf  \t %lf\t %lf\t %lf\t %lf\t %lf\t\n", x_dot_(0), x_dot_(1), x_dot_(2), x_dot_(3), x_dot_(4), x_dot_(5));

  fprintf(save_cmd, "%lf\t %d\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", simulation_time.toSec(), pin_state_,f_star_zero_(0), f_star_zero_(1), f_star_zero_(2), f_star_zero_(3), f_star_zero_(4), f_star_zero_(5));
  fprintf(save_fm, "%lf\t %d\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", simulation_time.toSec(), pin_state_,force_ee(0), force_ee(1), force_ee(2), moment_ee(0), moment_ee(1), moment_ee(2));

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

}


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerSyDualPin,
                       controller_interface::ControllerBase)
