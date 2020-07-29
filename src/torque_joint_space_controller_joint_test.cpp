#include <advanced_robotics_franka_controllers/torque_joint_space_controller_joint_test.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/model.h>


#include "math_type_define.h"
#include "peg_in_hole_base.h"
#include "peg_in_hole_base_2.h"
#include "criteria.h"

using namespace Criteria;
using namespace PegInHole;
using namespace DyrosMath;


namespace advanced_robotics_franka_controllers
{

bool TorqueJointSpaceControllerJointTest::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  joint0_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint0_data.txt","w");  
  pr_real = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/pr_real.txt","w");   
  fm_real = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/fm_real.txt","w");     
  fm_cmd = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/fm_cmd.txt","w");     
  torque_cmd = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/torque_cmd.txt","w");     
  spiral_position = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/spiral_position.txt","w");     
	
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

void TorqueJointSpaceControllerJointTest::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  pos_init_ = transform_init_.translation();	
  the_origin_ = transform_init_.translation();	
  ori_init_ = transform_init_.rotation();
  

  state_ = READY;
  assembly_dir_ = 2;

  is_ready_first_ = true;
  is_tilt_first_ = true;
  is_moveback_first_ = true;
  is_approach_first_ = true;
  is_search_first_ = true;
  is_insert_first_ = true;
  is_release_first_ = true;

  set_tilt_ = false;
  pause_ = true;

  ready_duration_ = 3.0;
  tilt_duration_ = 5.0;
  moveback_duration_ = 5.0;
  
  contact_check_cnt_ = 0;

  base_rotation_ << -0.438593, 0.897997, 0.0351727, 0.898657, 0.437932, 0.0251129, 0.00714807, 0.0426225, -0.99066;
  
  assembly_pos_ << 0.429348658, 0.860215723, 0.05;
  assembly_quat_ << 0, 0, 0.6613119, 0.7501111;
  grasp_pos_ << 0.30326221, 0.394805131, 0.029767764;
  // grasp_quat_ << 0, 0, -0.7071068, 0.7071068;
  // grasp_rot_ << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
  // grasp_rot_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  // grasp_rot_ << 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
  // grasp_rot_ << 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  // grasp_rot_ << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0;
  // grasp_rot_ << -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0;
  // grasp_rot_ << -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -1.0, 0.0;
    grasp_rot_ << 1.0, 0.0, 0.0, 0.0, 0.0, +1.0, 0.0, -1.0, 0.0;

  assembly_frame_ = PegInHole2::setTransformation(assembly_pos_, assembly_quat_);
  // grasp_frame_ = PegInHole2::setTransformation(grasp_pos_, grasp_quat_);
  grasp_frame_ = PegInHole2::setTransformation(grasp_pos_, grasp_rot_);

  std::cout<<"assembly_frame_: \n"<<assembly_frame_<<std::endl;
  std::cout<<"grasp_frame_: \n"<<grasp_frame_<<std::endl;

  f_ee_prev_.setZero();
  m_ee_prev_.setZero();

  f_asm_.setZero();
  
  gripperClose();
}


void TorqueJointSpaceControllerJointTest::update(const ros::Time& time, const ros::Duration& period) {

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
  // q_goal << 0, 0.0, 0.0, -M_PI/2, 0, M_PI/2, 0;
  // q_goal << 0.0, 0.0, 0.0, 0.0, 0.0, M_PI/3, 0;
  for(int i = 0; i < 7; i++)
  {
    q_goal(i) = q_init_(i);
    if( i == 5) q_goal(i) = q_init_(i)+M_PI/3;
  }
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

  // double trajectory_time = 15.0;
  // for(int i=0; i<7;i++)
  // {
  //   q_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
  //                                       q_init_(i), q_goal(i), 0, 0);
  //   qd_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
  //                                       q_init_(i), q_goal(i), 0, 0);
  // }
  // qd_desired.setZero();

//--------------------------code start frome here------------------------------------------
  Eigen::Matrix<double, 6, 6> lambda;
  Eigen::Matrix<double, 7, 6> J_bar;
  Eigen::Matrix<double, 6, 1> f_measured;
  Eigen::Vector3d force_ee; //w.r.t end-effector
  Eigen::Vector3d moment_ee; 
  Eigen::Matrix<double, 6, 1> xd;
  double f_reaction;

  lambda = (jacobian*mass_matrix.inverse()*jacobian.transpose()).inverse();
  J_bar = mass_matrix.inverse()*jacobian.transpose()*lambda;
  f_measured = J_bar.transpose()*(tau_measured - gravity); //w.r.t global frame

  force_ee = rotation_M.transpose()*f_measured.head<3>();
  moment_ee = rotation_M.transpose()*f_measured.tail<3>();

  force_ee = DyrosMath::lowPassFilter(force_ee, f_ee_prev_, 0.001, 0.0531);
  moment_ee = DyrosMath::lowPassFilter(moment_ee, m_ee_prev_, 0.001, 0.0531);  
  f_ee_prev_ = force_ee;
  m_ee_prev_ = moment_ee;

  xd = jacobian*qd;

  cur_time_ = time;

  double approach_threshold = 0.0;
    
  for(int i = 0; i < 3; i++)
  {
    if(i != assembly_dir_)
    {
      f_reaction += force_ee(i)*force_ee(i);
    }
  }

  f_reaction = sqrt(f_reaction); 
  
  gripperClose();



  switch(state_)
    {    
      case READY:
        ready(position, rotation_M, xd, f_measured, base_rotation_);
        if(timeOut(cur_time_.toSec(), start_time_.toSec(), tilt_duration_))
        {
          // state_ = TILT;
          // std::cout<<"holding_force!!: "<<holding_force_<<std::endl;
        }
        break;

      case TILT:
        tilt(position, xd, rotation_M, 45.0*M_PI/180, 0, tilt_duration_);
        if(timeOut(cur_time_.toSec(), tilt_start_time_.toSec(), tilt_duration_))
        {
          state_ = MOVEBACK;
          std::cout<<"TILT IS DONE"<<std::endl;
        }
        break;

      case MOVEBACK:
        moveback(position, xd, rotation_M, moveback_duration_);
        if(timeOut(cur_time_.toSec(), moveback_start_time_.toSec(), moveback_duration_))
        {
          state_ = APPROACH;
          std::cout<<"MOVEBACK IS DONE"<<std::endl;
        }
        break;

      case APPROACH:
        approach(position, xd, rotation_M);
        approach_threshold = -holding_force_/1.5;
        if(force_ee(assembly_dir_) > approach_threshold)
        {
          contact_check_cnt_++;
          if(getCount(contact_check_cnt_, 50))
          {
            state_ = SEARCH;
            std::cout<<"CONTACT IS DETECT!!"<<std::endl;
            std::cout<<"force_ee: "<<force_ee(assembly_dir_)<<std::endl;
            std::cout<<"threshold: "<<approach_threshold<<std::endl;
            contact_check_cnt_ = 0;
          }
          
        } 
        break;

      case SEARCH:
        search(position, rotation_M, xd);
        if(checkForceLimit(f_reaction, 10.0))
        {
          state_ = INSERT;
          std::cout<<"f_reaction: "<<f_reaction<<std::endl;
          std::cout<<"SEARCH IS COMPLETED!!"<<std::endl;
        }
        break;

      case INSERT:
        insert(position, rotation_M, xd);
        if(timeOut(cur_time_.toSec(), insert_start_time_.toSec(), 5.0))
        {
          state_ = RELEASE;
          std::cout<<"INSERTION IS DONE"<<std::endl;
        }
        break;

      case RELEASE:
        release(position, rotation_M, xd);
    }

  
  tau_cmd = jacobian.transpose() * (f_star_zero_);
  
//tau_cmd =  mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd));
// tau_cmd =  ( 800*(q_desired - q) + 5*(qd_desired - qd));


////  tau_cmd =  mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd));// + coriolis;
  //tau_cmd = -gravity + graivity_mod;
  // tau_cmd.setZero();

  if (print_rate_trigger_()) {
    // ROS_INFO("--------------------------------------------------");
    // ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    // ROS_INFO_STREAM("tau_measured :" << (tau_measured-gravity).transpose());
    // ROS_INFO_STREAM("f_measured :" << f_measured.head<3>().transpose() );
    // ROS_INFO_STREAM("force_ee :" << force_ee.transpose() );
    // ROS_INFO_STREAM("time :"<< simulation_time);
    // ROS_INFO_STREAM("position : "<< position.transpose());
    // ROS_INFO_STREAM("f_star_zero_ : "<< f_star_zero_.transpose());
    // ROS_INFO_STREAM("mass_matrix : "<<"\n"<< mass_matrix);
    // ROS_INFO_STREAM("rotation : "<<"\n"<< rotation_M);
  }

    fprintf(pr_real, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", position(0), position(1), position(2), rotation_M(0), rotation_M(1), rotation_M(2));
    fprintf(fm_real, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", force_ee(0), force_ee(1), force_ee(2), moment_ee(0), moment_ee(1), moment_ee(2));
    fprintf(fm_cmd, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_star_zero_(0), f_star_zero_(1), f_star_zero_(2), f_star_zero_(3), f_star_zero_(4), f_star_zero_(5));
    fprintf(torque_cmd, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", tau_cmd(0), tau_cmd(1), tau_cmd(2), tau_cmd(3), tau_cmd(4), tau_cmd(5), tau_cmd(6));
    if(state_ == SEARCH) fprintf(spiral_position, "%lf\t %lf\t %lf\t\n", position(0), position(1), position(2));
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

}

void TorqueJointSpaceControllerJointTest::ready(const Eigen::Vector3d position, const Eigen::Matrix3d rotation,
  const Eigen::Matrix<double, 6, 1> xd,
  const Eigen::Matrix<double, 6, 1> f_measured,
  const Eigen::Matrix3d base_rotation)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  double run_time;


  if(is_ready_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;
    tilt_start_time_ = cur_time_;
    is_ready_first_ = false;
    T_GA_ = PegInHole2::setTransformationInverse(grasp_frame_)* assembly_frame_;
    assembly_dir_vec_ = PegInHole2::getAssemblyDirction(T_GA_);
    set_tilt_ = PegInHole2::setTilt(T_GA_, assembly_dir_vec_, 0.01);
    tilt_axis_ = PegInHole2::getTiltDirection(T_GA_, assembly_dir_vec_);
    std::cout<<"T_GA: \n"<<T_GA_<<std::endl;
    std::cout<<"assembly_dir_: "<<assembly_dir_vec_.transpose()<<std::endl;
    // std::cout<<"set_tilt: "<<set_tilt_<<std::endl;
    std::cout<<"tilt_axis_!!!!!!!: "<<tilt_axis_.transpose()<<std::endl;
    std::cout<<"ori_init: \n"<<ori_init_<<std::endl;
    std::cout<<"ready"<<std::endl;
  }

  run_time = cur_time_.toSec() - start_time_.toSec();

  if(set_tilt_)
  {
    //f_star = PegInHole2::keepCurrentState(pos_init_, ori_init_, position, rotation, xd, 5000, 100).head<3>();
    //m_star = PegInHole2::rotateWithEeAxis(ori_init_, rotation, xd, 10*M_PI/180, cur_time_.toSec(), start_time_.toSec(), tilt_duration_, tilt_axis_);
    f_star = PegInHole2::tiltMotion(pos_init_, ori_init_, position, rotation, xd, T_GA_, tilt_axis_, 10*M_PI/180, cur_time_.toSec(), tilt_start_time_.toSec(), tilt_duration_).head<3>();
    m_star = PegInHole2::tiltMotion(pos_init_, ori_init_, position, rotation, xd, T_GA_, tilt_axis_, 10*M_PI/180, cur_time_.toSec(), tilt_start_time_.toSec(), tilt_duration_).tail<3>();
    // m_star = PegInHole2::keepCurrentState(pos_init_, ori_init_, position, rotation, xd, 5000, 100).tail<3>();
  }
  else
  {
    f_star = PegInHole2::keepCurrentState(pos_init_, ori_init_, position, rotation, xd, 5000, 100).head<3>();
    m_star = PegInHole2::keepCurrentState(pos_init_, ori_init_, position, rotation, xd, 5000, 100).tail<3>();
  }
  
  
  
  // std::cout<<"set tilt: "<<set_tilt_<<std::endl;
 
  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;

  if(pause_)
  {
    holding_force_ += f_measured(assembly_dir_);
    
    if(run_time > 0.25)
    {
      holding_force_ = holding_force_ / (run_time*1000);
    
      pause_ = false;
    }
  }
}

void TorqueJointSpaceControllerJointTest::tilt(const Eigen::Vector3d position, const Eigen::Matrix<double, 6, 1> xd, 
  const Eigen::Matrix3d rotation, const double tilt_angle, const int tilt_axis, const double duration) 
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  
  if(is_tilt_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;
    tilt_start_time_ = cur_time_;
    is_tilt_first_ = false;
    std::cout<<"tilt"<<std::endl;
  }

  
  // m_star = rotateWithEeAxis(ori_init_, rotation, xd, tilt_angle, tilt_start_time_.toSec(), cur_time_.toSec(), duration, tilt_axis);  
  // f_star = keepCurrentPosition(pos_init_, position, xd, 5000, 200);
  
  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerJointTest::moveback(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rotation, const double duration)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  
  if(is_moveback_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;
    moveback_start_time_ = cur_time_;
    is_moveback_first_ = false;
    std::cout<<"move back"<<std::endl;

  }

  f_star = twoDofMove(pos_init_, position, the_origin_, xd, cur_time_.toSec(), moveback_start_time_.toSec(), duration, 0.0, assembly_dir_);
  m_star = keepCurrentOrientation(ori_init_, rotation, xd);
  
  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;

}

void TorqueJointSpaceControllerJointTest::approach(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rotation)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  if(is_approach_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;
    search_pose_rotation_ = rotation;
    approach_start_time_ = cur_time_;
    is_approach_first_ = false;
    std::cout<<"start approach"<<std::endl;
  }
  
  
  f_star = straightMoveEE(pos_init_, position, xd, assembly_dir_, 0.005, cur_time_.toSec(), approach_start_time_.toSec(), ori_init_);
  // f_star = straightMove(pos_init_, position, xd, assembly_dir_, -0.005, cur_time_.toSec(), approach_start_time_.toSec());
    
  m_star = keepCurrentOrientation(ori_init_, rotation, xd);
  
  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
  }

void TorqueJointSpaceControllerJointTest::search(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  double pitch;
  double lin_v;
  double duration = 3000.0;
  pitch = 0.001;
  lin_v = 0.008;

  if(is_search_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;
    spiral_start_time_ = cur_time_;
    is_search_first_ = false;
    std::cout<<"start search"<<std::endl;
  }

  f_star = generateEllipseSpiralEE(pos_init_, position, xd, ori_init_, pitch, lin_v, assembly_dir_, cur_time_.toSec(), spiral_start_time_.toSec(), duration, 0.8, 1.0);
  Eigen::Vector3d temp = ori_init_*f_asm_;
  f_star(assembly_dir_) += temp(assembly_dir_);
  
  m_star = keepCurrentOrientation(search_pose_rotation_, rotation, xd, 300, 2);
  
  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;

  if(f_asm_(assembly_dir_)<= 20.0) f_asm_(assembly_dir_) += 0.0001;
  else f_asm_(assembly_dir_) = f_asm_(assembly_dir_);
}

void TorqueJointSpaceControllerJointTest::insert(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Vector3d f_asm;

  f_asm << 0, 0, 5.0;

  if(is_insert_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;

    insert_start_time_ = cur_time_;

    is_insert_first_ = false;
    std::cout<<"start insert"<<std::endl;
  }
  f_asm = ori_init_*f_asm;

  f_star = keepCurrentState(pos_init_, ori_init_, position, rotation, xd, 5000, 100).head<3>();
  for(int i = 0; i < 3; i++)  f_star(i) += f_asm(i);
  m_star = keepCurrentOrientation(ori_init_, rotation, xd);
  
  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerJointTest::release(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  if(is_release_first_)
  {
    pos_init_ = position;
    ori_init_ = rotation;
    release_start_time_ = cur_time_;
    is_release_first_ = false;
    std::cout<<"start release"<<std::endl;
  }

  f_star = keepCurrentState(pos_init_, ori_init_, position, rotation, xd, 5000, 100).head<3>();
  // m_star = rotateUsingMatrix(ori_init_, rotation, xd, base_rotation_, release_start_time_.toSec(), cur_time_.toSec(), tilt_duration_);
  m_star = rotateUsingMatrix(ori_init_, rotation, xd, ori_init_, release_start_time_.toSec(), cur_time_.toSec(), tilt_duration_);

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerJointTest::gripperClose()
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


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerJointTest,
                       controller_interface::ControllerBase)
