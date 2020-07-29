#include <advanced_robotics_franka_controllers/torque_joint_space_controller_fuzzy.h>
#include <cmath>
#include <cstdlib>
#include <ctime>
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
#include "peg_in_hole_base.h"
#include "criteria.h"

using namespace Criteria;
using namespace PegInHole;

namespace advanced_robotics_franka_controllers
{

bool TorqueJointSpaceControllerFuzzy::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  gripper_ac_close_.waitForServer();
  gripper_ac_open_.waitForServer();

  //joint0_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint0_data.txt","w");
  fuzzy_io = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/ur/fuzzy_io.txt","w");

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

  Eigen::Vector3d sgn;
  srand((unsigned int)time(NULL));
  for(size_t i = 0; i < 3; i++)
  {
    if(rand() % 2 == 0) sgn(i) = 1;
    else sgn(i) = -1;
  }
  
  std::cout<<"sgn: "<<sgn.transpose()<<std::endl;
  srand((unsigned int)time(NULL));
  for(size_t i = 0; i < 3; i++)
  {
    pos_random_(i) = rand() % 16 + 0;
    pos_random_(i) = sgn(i)*(pos_random_(i) / 1000);
  }
  
  // std::cout<<"error: "<<pos_random_.transpose()<<std::endl;

  return true;
}

void TorqueJointSpaceControllerFuzzy::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  pos_init_ = transform_init_.translation();
  origin_	<< 0.6607, 0.2542, 0.08;
  ori_init_ = transform_init_.rotation();

  // std::cout<<"origin_: "<<origin_.transpose()<<std::endl;
  // std::cout<<"pos_init_: "<<pos_init_.transpose()<<std::endl;
  xdot_desired_.setZero();
 
  finish_time = time - start_time_;

  status_ = -1;
  assembly_dir_ = 2; // z-axis w.r.t EE

  is_ready_ = false;

  is_random_first_ = true;
  is_approach_first_ = true;
  is_search_first_ = true;
  is_insert_first_ = true;
  is_release_first_ = true;
  is_escape_first_ = true;
  is_back_first_ = true;

  is_random_done_ = false;
  is_approach_done_ = false;
  is_search_done_ = false;
  is_insert_done_ = false;
  is_escape_done_ = false;
  is_back_done_ = false;
  is_release_done_ = false;
  

  fuzzy_output_ = 0;
  fuzzy_output_prev_ = 0;
  crisp_output_ = 0;
  crisp_output_prev_ = 0;
  count_ = 0;
  // gripperClose();

  // std::ifstream test_set;
  // test_set.open("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/ur/cs_5_1/05_cs_5_1_fail/fuzzy_io.txt");
  // for(int i =0; i<test_set_size_; i++)
  // {
  //   test_set >> a[i] >> xe[i] >> ye[i] >> v[i] >> z[i] >> f_ee[i] >> a1[i] >> a2[i] >> a3[i] >> a4[i];
  // }
  // // std::cout<<"data size: "<<z.size()<<std::endl;
  // test_set.close();
  // tic_ = 0.0;
}


void TorqueJointSpaceControllerFuzzy::update(const ros::Time& time, const ros::Duration& period) {

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

  ros::Duration simulation_time = time - start_time_;
  Eigen::Matrix<double, 7, 1> tau_cmd;
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation()); 

  Eigen::Vector6d x_dot_(jacobian*qd);

  jacobian_pos_ = jacobian.block(0, 0, 3, 7);
  Eigen::Vector6d xd;
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

  double approach_threshold = -6.0;

  xd = jacobian*qd;

  double f = 0.0;
  for(size_t i = 0; i < 3; i++)
  {
    if(i != assembly_dir_) f += pow(force_ee(i), 2);
  }
  f = sqrt(f);
  
  cur_time_ = time;
  
  
///////////////////////////////////////////////

  // franka_gripper::MoveGoal goal;
  // goal.speed = 0.02;
  // goal.width = 0.007; //0.001
  // gripper_ac_.sendGoal(goal);
  // gripper_ac_.waitForServer();

  // franka_gripper::GraspGoal goal;

  // if (check_gripper == 0)
  // {
  //   goal.speed = 0.01;
  //   goal.width = 0.017;
  //   goal.force = 60.0;
  //   gripper_grasp_.sendGoal(goal);
  //   check_gripper = 1;
  // }

/////////////////////////////////////////////////
// status_ = SEARCH;


  switch(status_)
  {
    case READY:
      moveToRandomPoint(position, rotation_M, xd);
      if(is_random_done_ == true)
      {
        status_ = APPROACH;
        std::cout<<"START PEG IN HOLE"<<std::endl;
      }
      break;
      
    case APPROACH:
      approach(position, xd, rotation_M);
      if(checkContact(f_measured(assembly_dir_), approach_threshold))
      {
        status_ = SEARCH;
        is_approach_done_ = true;
        std::cout<<"CONTACT IS DETECT!!"<<std::endl;
      } 
      break;

    case SEARCH:
      search(position, rotation_M, xd);
      
      // pos_init_(assembly_dir_) = z[0];
      // xd(assembly_dir_) = v[tic_];
      // position(assembly_dir_) = z[tic_];
      // f = f_ee[tic_];

      fuzzy_output_cur_ = fuzzyLogic(pos_init_(assembly_dir_), xd(assembly_dir_), position(assembly_dir_), f);
      crisp_output_cur_ = crispLogic(pos_init_(assembly_dir_), xd(assembly_dir_), position(assembly_dir_), f);

      if(abs(fuzzy_output_prev_ - fuzzy_output_cur_) < 0.001)
      {
        count_++;
        if(count_ >= 20)
        { 
          fuzzy_output_ = fuzzy_output_cur_;
          
          if(fuzzy_output_ == NONE || fuzzy_output_ == CS_ONE || fuzzy_output_ == CS_TWO) status_ = status_;
          if(fuzzy_output_ == CS_FOUR) status_ = INSERT;
          if(fuzzy_output_ == CS_THREE || fuzzy_output_ == CS_FIVE_ONE || fuzzy_output_ == CS_FIVE_TWO) status_ = ESCAPE;
        //   switch(fuzzy_output_)
        //   {
        //     case NONE:
        //       status_ = status_;
        //       // std::cout<<"NONE"<<std::endl;
        //       break;
        //     case CS_ONE:
        //       status_ = status_;
        //       // std::cout<<"CS_ONE"<<std::endl;
        //       break;
        //     case CS_TWO:
        //       status_ = status_;
        //       // std::cout<<"CS_TWO"<<std::endl;
        //       break;
        //     case CS_THREE:
        //       status_ = ESCAPE;
        //       // std::cout<<"CS_THREE"<<std::endl;
        //       break;
        //     case CS_FOUR:
        //       status_ = INSERT;
        //       // std::cout<<"CS_FOUR"<<std::endl;
        //       break;
        //     case CS_FIVE_ONE:
        //       status_ = ESCAPE;
        //       // std::cout<<"CS_FIVE_ONE"<<std::endl;
        //       break;
        //     case CS_FIVE_TWO:
        //       status_ = ESCAPE;
        //       // std::cout<<"CS_FIVE_TWO"<<std::endl;
        //       break;
        //   }
        // }
        }
      }
      else count_ = 0;
    

      if(crisp_output_prev_ == crisp_output_cur_)
      {
        count_2_++;
        if(count_2_ > 20) crisp_output_ = crisp_output_cur_;
      }
      else count_2_ = 0;

      fuzzy_output_prev_ = fuzzy_output_cur_;
      crisp_output_prev_ = crisp_output_cur_;

      break;

    case INSERT:
      insert(position, rotation_M, xd);
      if(timeOut(cur_time_.toSec(), insert_start_time_.toSec(), 2.0))
      {
        status_ = BACK;
        std::cout<<cur_time_.toSec() - insert_start_time_.toSec()<<std::endl;
        std::cout<<"INSERTION IS DONE"<<std::endl;
      }
      f_star_zero_.setZero();
      break;
    
    case BACK:
      back(position, rotation_M, xd);
      if(is_back_done_)
      {
        status_ = RELEASE;
        std::cout<<"RETURN THE ORIGIN"<<std::endl;
      }
      f_star_zero_.setZero();
      break;

    case ESCAPE:
      escape(position, rotation_M, xd);
      if(is_escape_done_)
      {
        status_ = RELEASE;
        std::cout<<"RETURN THE ORIGIN"<<std::endl;
      }
      f_star_zero_.setZero();
      break;

    case RELEASE:
      // release(position, xd, rotation_M);
      // if(is_release_done_)
      // {
      //   // std::cout<<"READY TO START AGIN!!"<<std::endl;
      // } 
      // break;   
      f_star_zero_.setZero();
  }

  // tic_++;
  // std::cout<<"ic_: "<<tic_<<std::endl;

  tau_cmd = jacobian.transpose() * (f_star_zero_);
 
  ////////////////////


  //tau_cmd.setZero();

  if (print_rate_trigger_()) {
    // ROS_INFO("--------------------------------------------------");
    // ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    // //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    // //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    // ROS_INFO_STREAM("time :"<< simulation_time);
    // ROS_INFO_STREAM("x_curent : "<< position.transpose());
    // ROS_INFO_STREAM("x_desired : "<< x_desired_.transpose());
    // //ROS_INFO_STREAM("mass :" << mass_matrix);
    // ROS_INFO_STREAM("check_fake :" << check_fake);
    //  ROS_INFO_STREAM("q :"<< q.transpose());
    // ROS_INFO_STREAM("f_sensing : "<< f_sensing.transpose());
    // ROS_INFO_STREAM("rotation : "<< rotation_M);
    // ROS_INFO_STREAM("euler_angle_ : "<< euler_angle_.transpose());

    // ROS_INFO_STREAM("finish_time : "<< finish_time);

  }

  //fprintf(joint0_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", time.toSec(), q_desired(0), q(0), qd(0), tau_cmd(0), tau_J_d(0), tau_measured(0), mass_matrix(0, 0));
  if(status_ >= SEARCH)
      fprintf(fuzzy_io, "%d\t %lf\t %lf\t %lf\t %lf\t %lf\t %f\t %f\t %f\t %f\t\n", status_, pos_random_(0), pos_random_(1), xd(assembly_dir_), position(assembly_dir_), f, fuzzy_output_cur_, fuzzy_output_, crisp_output_cur_, crisp_output_);
  
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

}

void TorqueJointSpaceControllerFuzzy::approach(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rotation)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  if(is_approach_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;
    approach_start_time_ = cur_time_;
    is_approach_first_ = false;
    std::cout<<"start approach"<<std::endl;
  }
  
  f_star = straightMove(pos_init_, position, xd, assembly_dir_, -0.005, cur_time_.toSec(), approach_start_time_.toSec());
  m_star = keepOrientationPerpenticular(ori_init_, rotation, xd, 1.0, cur_time_.toSec(), approach_start_time_.toSec());
  
  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerFuzzy::search(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  double pitch;
  double lin_v;
  double duration = 1000.0;
  double ori_duration = 1.0;
  pitch = 0.001;
  lin_v = 0.01;
  
  if(is_search_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;

    spiral_start_time_ = cur_time_;

    is_search_first_ = false;
    std::cout<<"start search"<<std::endl;
  }

  f_star = generateSpiral(pos_init_, position, xd, pitch, lin_v, assembly_dir_, cur_time_.toSec(), spiral_start_time_.toSec(), duration);
  f_star(assembly_dir_) = -6.0;
  
  m_star = keepOrientationPerpenticular(ori_init_, rotation, xd, 1.0, cur_time_.toSec(), spiral_start_time_.toSec());


  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerFuzzy::insert(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  if(is_insert_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;

    insert_start_time_ = cur_time_;

    is_insert_first_ = false;
    std::cout<<"start insert"<<std::endl;
  }

  f_star = keepCurrentState(pos_init_, ori_init_, position, rotation, xd, 5000, 100).head<3>();
  f_star(assembly_dir_) = -15.0;

  m_star = keepOrientationPerpenticular(ori_init_, rotation, xd, 1.0, cur_time_.toSec(), insert_start_time_.toSec());
  
  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerFuzzy::back(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  
  double duration  = 5.0;

  if(is_back_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;

    back_start_time_ = cur_time_;
    std::cout<<"start back"<<std::endl;
    is_back_first_ = false;
  }
  
  double run_time = cur_time_.toSec() - escape_start_time_.toSec();

  if(run_time < duration)
  {
    f_star = oneDofMove(pos_init_, position, origin_(assembly_dir_), xd, cur_time_.toSec(), escape_start_time_.toSec(), duration, 0.1, assembly_dir_);
    std::cout<<run_time<<std::endl;
  }
    
  else
    is_back_done_ = true;

  m_star = keepCurrentOrientation(ori_init_, rotation, xd, 200, 5);

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerFuzzy::escape(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  
  double move_up_duration = 5.0;
  double go_to_origin_duration = 5.0;

  bool move_up_is_done = false;

  if(is_escape_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;

    escape_start_time_ = cur_time_;
    
    std::cout<<"start escape"<<std::endl;
    is_escape_first_ = false;
  }

  double run_time = cur_time_.toSec() - escape_start_time_.toSec();

  if(run_time > move_up_duration)
  {
    move_up_is_done = true;
  } 
  if(move_up_is_done)
  {
    pos_init_ = pos_init_;
    pos_init_(assembly_dir_) = origin_(assembly_dir_);
  }

  
  if(run_time <= move_up_duration)
  {
    f_star = oneDofMove(pos_init_, position, origin_(assembly_dir_), xd, cur_time_.toSec(), escape_start_time_.toSec(), move_up_duration, 0.1, assembly_dir_);
    // std::cout<<"move up"<<std::endl;
  }
  else if(move_up_duration < run_time && run_time <= move_up_duration + go_to_origin_duration)
  {
    f_star = twoDofMove(pos_init_, position, origin_, xd, cur_time_.toSec(), escape_start_time_.toSec() + move_up_duration, go_to_origin_duration, 0.05, assembly_dir_);
  }
  else
  {
    is_escape_done_ = true;
  }
  
  m_star = keepCurrentOrientation(ori_init_, rotation, xd, 200, 5);

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}


void TorqueJointSpaceControllerFuzzy::release(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rotation)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  double duration = 5.0;

  if(is_release_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;

    init_force_ = f_star_zero_.head<3>();
    goal_force_.setZero();

    init_moment_ = f_star_zero_.tail<3>();
    goal_moment_.setZero();

    release_start_time_ = cur_time_;
    // release_start_time_.toSec() + duration;

    is_release_first_ = false;
    std::cout<<"start release"<<std::endl;
  }

  if(cur_time_.toSec() - release_start_time_.toSec() > duration)
  {
    is_release_done_ = true;
    // is_release_first_ = true;
    // std::cout<<"RELEASE IS DONE, GO TO THE NEXT STEP"<<std::endl;
  } 

  for(size_t i = 0; i < 3; i ++)
  {
    f_star(i) = cubic(cur_time_.toSec(), release_start_time_.toSec(), release_start_time_.toSec() + duration, init_force_(i), goal_force_(i), 0, 0);
    m_star(i) = cubic(cur_time_.toSec(), release_start_time_.toSec(), release_start_time_.toSec() + duration, init_moment_(i), goal_moment_(i), 0, 0);
  } 

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerFuzzy::moveToRandomPoint(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
    
  double duration = 5.0;
  double run_time = cur_time_.toSec() - start_time_.toSec();

  if(is_random_first_)
  {
    Eigen::Vector3d temp;
    // goal_ = pos_init_ + pos_random_;
    goal_ = pos_init_;
  
    goal_(assembly_dir_) = pos_init_(assembly_dir_);

    std::cout<<"origin_: "<<origin_.transpose()<<std::endl;
    std::cout<<"pos_init_: "<<pos_init_.transpose()<<std::endl;
    std::cout<<"random pos: "<<goal_.transpose()<<std::endl;
    std::cout<<"positio error: "<<pos_random_.transpose()<<std::endl;
    std::cout<<"start random"<<std::endl;

    is_random_first_ = false;
  }

  if(run_time <= duration)
  {
    f_star = twoDofMove(pos_init_, position, goal_, xd, cur_time_.toSec(), start_time_.toSec(), duration, 0.01, assembly_dir_);
    
    // std::cout<<"------------------------------------------"<<std::endl;
    // std::cout<<"init_position: "<<pos_init_.transpose()<<std::endl;
    // std::cout<<"current position: "<<position.transpose()<<std::endl;    
    // std::cout<<"goal_ position: "<<goal_.transpose()<<std::endl;
    // std::cout<<"f_star: "<<f_star.transpose()<<std::endl;

  }
  else
  {
    is_random_done_ = true;
  }  

  m_star = keepCurrentOrientation(ori_init_, rotation, xd, 200, 5);

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerFuzzy::gripperClose()
{
    franka_gripper::GraspGoal goal;
    franka_gripper::GraspEpsilon epsilon;
    epsilon.inner = 0.01;
    epsilon.outer = 0.02;
    goal.speed = 0.01;
    goal.width = 0.03;
    goal.force = 100.0;
    goal.epsilon = epsilon;
    gripper_ac_close_.sendGoal(goal);
}

void TorqueJointSpaceControllerFuzzy::gripperOpen()
{
  franka_gripper::MoveGoal goal;
  goal.speed = 0.020;
  goal.width = 0.01;
  gripper_ac_open_.sendGoal(goal);
}


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerFuzzy,
                       controller_interface::ControllerBase)