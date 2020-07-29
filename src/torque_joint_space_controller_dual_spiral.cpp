#include <advanced_robotics_franka_controllers/torque_joint_space_controller_dual_spiral.h>
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
#include "peg_in_hole_base.h"
#include "peg_in_hole_base_2.h"
#include "criteria.h"

using namespace Criteria;
using namespace PegInHole;

namespace advanced_robotics_franka_controllers
{

bool TorqueJointSpaceControllerDualSpiral::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  //joint0_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint0_data.txt","w");
  save_data_x = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_data_x.txt","w");   
  save_data_x2 = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_data_x2.txt","w");   
  force_moment_ee = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/force_moment_ee.txt","w");
  cmd_task_space = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/cmd_task_space.txt","w");   
  cmd_joint_space = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/cmd_joint_space.txt","w");
  //save_force = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/save_force.txt","w");   
  //save_position = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/save_position.txt","w");   
  //save_velocity = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/save_velocity.txt","w");   
  
  gripper_ac_.waitForServer();

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

void TorqueJointSpaceControllerDualSpiral::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  pos_init_ = transform_init_.translation();	
  ori_init_ = transform_init_.rotation();
//  ori_init_ = transform_init_.linear();
//  ori_init_ << 0.7071068, 0.7071068, 0.0, 0.7071068, -0.7071068, 0.0, 0.0, 0.0, -1.0;
  xdot_desired_.setZero();

  check_stop = 0;
  check_fake = 0;
  check_contact = false;

  // franka_gripper::MoveGoal goal;
  // goal.speed = 0.02;
  // goal.width = 0.001;
  // gripper_ac_.sendGoal(goal);
  // ROS_INFO("Gripper goal published");
  
  // franka_gripper::GraspGoal goal;
  // goal.speed = 0.1;
  // goal.width = 0.001;
  // goal.force = 0.5;
  // franka_gripper::grasp(franka::Gripper gripper, goal);

  //gripper_grasp_

  // franka_gripper::GraspGoal goal;
  // goal.speed = 0.02;
  // goal.width = 0.007;
  // goal.force = 20.0;
  // gripper_grasp_.sendGoal(goal);
  // ROS_INFO("Gripper goal published");

  //franka_gripper::HomingAction home_goal;
  //gripper_homing_;

  check_gripper = 0;

  //target_rotation_ << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  //target_rotation_ << 0, 0, 1, 0, 1, 0, -1, 0, 0;
  target_rotation_ = ori_init_;
  ori_theta = 0.0;
  ori_theta_init = 0.0;
  ori_change_direction = 0;
  ori_check_time = 0;
  ori_duration = 1.0;
  
  //finish_time = time - start_time_;

  //finish_time= time + time + time + time + time;

  f_sensing_read.setZero();
  f_sensing_read_check = 0;

  K_p.setZero(); K_v.setZero();
	for (int i = 0; i < 3; i++)
	{
		K_p(i, i) = 5000.0; K_v(i, i) = 100.0; //7000
	}
  K_p(2, 2) = 3000.0; //5000

  xdot_desired_.setZero();

  rotation_z_theta_.setZero();
  rotation_z_theta_2_.setZero();
  
  status_ = 0;
  assembly_dir_ = 2; // z-axis w.r.t EE

  is_approach_first_ = true;
  is_search_first_ = true;
  is_insert_first_ = true;
  is_release_first_ = true;

  is_approach_done_ = false;
  is_search_done_ = false;
  is_insert_done_ = false;
  is_release_done_ = false;

  assembly_pos_ << -0.135, 0.0, 0.01;
  assembly_quat_ << -0.7071068, 0, 0.7071068, 0;
  grasp_pos_ << 0.0, 0.0, 0.01;
  // grasp_quat_ << 
  grasp_rot_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  assembly_frame_ = PegInHole2::setTransformation(assembly_pos_, assembly_quat_);
  grasp_frame_ = PegInHole2::setTransformation(grasp_pos_, grasp_rot_);

  T_GA_ = PegInHole2::setTransformationInverse(grasp_frame_)*assembly_frame_;
  T_EA_ = T_GA_;

  assembly_dir_vec_ = PegInHole2::getAssemblyDirction(T_GA_);
  std::cout<<"T_GA: \n"<<T_GA_<<std::endl;
  std::cout<<"assembly_dir_vec_: "<<assembly_dir_vec_.transpose()<<std::endl;

}


void TorqueJointSpaceControllerDualSpiral::update(const ros::Time& time, const ros::Duration& period) {

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
  
//   q_goal.setZero();
//   q_goal << 0.0, -M_PI/6, 0.0, -2*M_PI/3, 0, M_PI/2, M_PI/4;
// //    q_goal << 0.0, 0.0, 0.0, 0.0, 0, 0.0, M_PI/4;
//   q_desired.setZero();

  //q_goal = q_init_;
  //q_goal(5) -= M_PI/6.0;

  ros::Duration simulation_time = time - start_time_;
  Eigen::Matrix<double, 7, 1> tau_cmd;
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation());
//  Eigen::Matrix<double, 3, 3> rotation_M(transform.linear());

  Eigen::Vector6d x_dot_(jacobian*qd);

  jacobian_pos_ = jacobian.block(0, 0, 3, 7);


  double trajectory_time = 5.0;

  Eigen::Vector6d xd;
  Eigen::Matrix<double, 6, 1> f_star_zero;
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

  double threshold = -6.0;

  xd = jacobian*qd;
  
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
//status_ = 3;
  switch(status_)
  {
    case 0:
      approach(position, xd, rotation_M);
      if(checkContact(f_measured(assembly_dir_), threshold))
      {
        status_ = 1;
        is_approach_done_ = true;
        std::cout<<force_ee(assembly_dir_ee_)<<std::endl;
        std::cout<<"CONTACT IS DETECT!!"<<std::endl;
      } 
      break;
    case 1:
      search(position, rotation_M, xd);
      if(pos_init_(2) - position(2) > 0.002) //w.r.t the global frame
      {
        status_ = 2;
        is_search_done_ = true;
        std::cout<<"Hole IS DETECTED"<<std::endl;
      }
      break;
    case 2:
      insert(position, rotation_M, xd);
      if(cur_time_.toSec() - insert_start_time_.toSec() > 3.0)
      {
        status_ = 3;
        is_insert_done_ = true;
        std::cout<<"INSERTION IS DONE"<<std::endl;
      }
      break;
    case 3:
      release(position, xd, rotation_M);
      if(is_release_done_)
      {
        status_ = 4;
        std::cout<<"READY TO OPEN A GRIPPER"<<std::endl;
      } 
      break;   
    
    case 4:
      gripperOpen();
      break;

  }

  tau_cmd = jacobian.transpose() * (f_star_zero_);

  
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
    // ROS_INFO_STREAM("q :"<< q.transpose());
    // ROS_INFO_STREAM("f_sensing : "<< f_measured.transpose());
    // ROS_INFO_STREAM("rotation : \n"<< rotation_M);

    // ROS_INFO_STREAM("finish_time : "<< finish_time);

  }

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

  fprintf(force_moment_ee, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", force_ee(0), force_ee(1), force_ee(2), moment_ee(0), moment_ee(1), moment_ee(2));
  fprintf(save_data_x2, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", position(0), position(1), position(2), x_dot_(0), x_dot_(1), x_dot_(2));
  fprintf(save_data_x, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_measured(0), f_measured(1), f_measured(2), f_measured(3), f_measured(4), f_measured(5));
  fprintf(cmd_task_space, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_star_zero_(0), f_star_zero_(1), f_star_zero_(2), f_star_zero_(3), f_star_zero_(4), f_star_zero_(5));
  fprintf(cmd_joint_space, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", tau_cmd(0), tau_cmd(1), tau_cmd(2), tau_cmd(3), tau_cmd(4), tau_cmd(5), tau_cmd(6));

}

void TorqueJointSpaceControllerDualSpiral::approach(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rotation)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  if(is_approach_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;

    is_approach_first_ = false;
    std::cout<<"approach first"<<std::endl;
  }
  
   //Be ee frame!!!
  //f_star = straightMoveEE(pos_init_, position, xd, assembly_dir_ee_, 0.005, cur_time_.toSec(), start_time_.toSec(), ori_init_);
  f_star = straightMove(pos_init_, position, xd, assembly_dir_, -0.005, cur_time_.toSec(), start_time_.toSec());
  m_star = keepOrientationPerpenticular(ori_init_, rotation, xd, 1.0, cur_time_.toSec(), start_time_.toSec());

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerDualSpiral::search(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  double pitch;
  double lin_v;
  double duration = 100.0;
  double ori_duration = 1.0;
  pitch = 0.001;
  lin_v = 0.01;
  
  if(is_search_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;

    spiral_start_time_ = cur_time_;

    is_search_first_ = false;
    std::cout<<"search first"<<std::endl;
  }

  // f_star = generateSpiral(pos_init_, position, xd, pitch, lin_v, assembly_dir_, cur_time_.toSec(), spiral_start_time_.toSec(), duration);
  f_star = PegInHole2::generateSpiralEE(pos_init_, ori_init_, position, xd, pitch, lin_v, T_EA_, cur_time_.toSec(), spiral_start_time_.toSec(), duration);
  f_star(assembly_dir_) = -1.0;
  // f_star += PegInHole2::press(ori_init_, assembly_dir_vec_, 1.0);

  if (ori_change_dir_ == 0)
  {
    if (is_first_ == true)
    {
      ori_start_time_ = cur_time_;
      is_first_ = false;
      // ori_init_ = desired_rotation_M;
    }  

    m_star = generateSpiralWithRotation(ori_init_, rotation, xd.tail<3>(), cur_time_.toSec(), ori_start_time_.toSec(), ori_duration, ori_change_dir_, assembly_dir_, 3*M_PI/180);

    if (cur_time_.toSec() > ori_start_time_.toSec() + ori_duration / 2)
    {
      ori_change_dir_ = 1;
      is_first_ = true;
    }
  }

  if (ori_change_dir_ == 1)
  {
    if (is_first_ == true)
    {
      ori_start_time_ = cur_time_;
      is_first_ = false;
    }
    
    m_star = generateSpiralWithRotation(ori_init_, rotation, xd.tail<3>(), cur_time_.toSec(), ori_start_time_.toSec(), ori_duration, ori_change_dir_, assembly_dir_, 3*M_PI/180);

    if (cur_time_.toSec() > ori_start_time_.toSec() + ori_duration)
    {
      ori_change_dir_ = 2;
      is_first_ = true;
    }
  }

  if (ori_change_dir_ == 2)
  {
    if (is_first_ == true)
    {
      ori_start_time_ = cur_time_;
      is_first_ = false;
    }

    m_star = generateSpiralWithRotation(ori_init_, rotation, xd.tail<3>(), cur_time_.toSec(), ori_start_time_.toSec(), ori_duration, ori_change_dir_, assembly_dir_, 3*M_PI/180);

    if (cur_time_.toSec() > ori_start_time_.toSec() + ori_duration)
    {
      ori_change_dir_ = 1;
      is_first_ = true;
    }
  }

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerDualSpiral::insert(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  double ori_duration = 1.0;

  if(is_insert_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;

    insert_start_time_ = cur_time_;

    is_insert_first_ = false;
    std::cout<<"insert first"<<std::endl;
  }

  f_star = keepCurrentState(pos_init_, ori_init_, position, rotation, xd, 5000, 100).head<3>();
  f_star(assembly_dir_) = -10.0;
  
  if (ori_change_dir_ == 0)
  {
    if (is_first_ == true)
    {
      ori_start_time_ = cur_time_;
      is_first_ = false;
      // ori_init_ = desired_rotation_M;
    }  

    m_star = generateSpiralWithRotation(ori_init_, rotation, xd.tail<3>(), cur_time_.toSec(), ori_start_time_.toSec(), ori_duration, ori_change_dir_, assembly_dir_, 0*M_PI/180);

    if (cur_time_.toSec() > ori_start_time_.toSec() + ori_duration / 2)
    {
      ori_change_dir_ = 1;
      is_first_ = true;
    }
  }

  if (ori_change_dir_ == 1)
  {
    if (is_first_ == true)
    {
      ori_start_time_ = cur_time_;
      is_first_ = false;
    }
    
    m_star = generateSpiralWithRotation(ori_init_, rotation, xd.tail<3>(), cur_time_.toSec(), ori_start_time_.toSec(), ori_duration, ori_change_dir_, assembly_dir_, 0*M_PI/180);

    if (cur_time_.toSec() > ori_start_time_.toSec() + ori_duration)
    {
      ori_change_dir_ = 2;
      is_first_ = true;
    }
  }

  if (ori_change_dir_ == 2)
  {
    if (is_first_ == true)
    {
      ori_start_time_ = cur_time_;
      is_first_ = false;
    }

    m_star = generateSpiralWithRotation(ori_init_, rotation, xd.tail<3>(), cur_time_.toSec(), ori_start_time_.toSec(), ori_duration, ori_change_dir_, assembly_dir_, 0*M_PI/180);

    if (cur_time_.toSec() > ori_start_time_.toSec() + ori_duration)
    {
      ori_change_dir_ = 1;
      is_first_ = true;
    }
  }

  //m_star.setZero();
  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerDualSpiral::release(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rotation)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  double duration = 2.0;

  if(is_release_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;

    init_force_ = f_star_zero_.head<3>();
    // goal_force_ = keepCurrentState(pos_init_, ori_init_, position, rotation, xd, 5000, 100).tail<3>();
    // goal_force_(assembly_dir_) = -6.0;
    goal_force_.setZero();

    init_moment_ = f_star_zero_.tail<3>();
    goal_moment_.setZero();

    release_start_time_ = cur_time_;
    release_start_time_.toSec() + duration;

    is_release_first_ = false;
    std::cout<<"init_force_: "<<init_force_.transpose()<<std::endl;
    std::cout<<"release first"<<std::endl;
  }

  if(cur_time_.toSec() - release_start_time_.toSec() > duration)
  {
    is_release_done_ = true;
    is_release_first_ = true;
    std::cout<<cur_time_.toSec() - release_start_time_.toSec()<<std::endl;
    std::cout<<"RELEASE IS DONE, GO TO THE NEXT STEP"<<std::endl;

  } 

  for(size_t i = 0; i < 3; i ++)
  {
    f_star(i) = cubic(cur_time_.toSec(), release_start_time_.toSec(), release_start_time_.toSec() + duration, init_force_(i), goal_force_(i), 0, 0);
    m_star(i) = cubic(cur_time_.toSec(), release_start_time_.toSec(), release_start_time_.toSec() + duration, init_moment_(i), goal_moment_(i), 0, 0);
  } 

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
  
}

void TorqueJointSpaceControllerDualSpiral::gripperOpen()
{
  franka_gripper::MoveGoal goal;
  goal.speed = 0.015;
  goal.width = 0.1;
  gripper_ac_.sendGoal(goal);
}

} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerDualSpiral,
                       controller_interface::ControllerBase)
