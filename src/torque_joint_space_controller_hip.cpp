#include <advanced_robotics_franka_controllers/torque_joint_space_controller_hip.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/model.h>

#include "math_type_define.h"
#include "peg_in_hole_base.h"
#include "criteria.h"

using namespace Criteria;
using namespace PegInHole;
using namespace DyrosMath;

namespace advanced_robotics_franka_controllers
{

bool TorqueJointSpaceControllerHip::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  joint0_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint0_data.txt","w");  
  force_moment_ee = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/force_moment_ee.txt","w");
  vel_ang_ee = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/vel_ang_ee.txt","w");
  force_select = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/force_select.txt","w");
  pos_ee = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/pos_ee.txt","w");

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

void TorqueJointSpaceControllerHip::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  pos_init_ = transform_init_.translation();	
  ori_init_ = transform_init_.rotation();

  assemble_dir_ = 0; //Set assemble direction is x-axis with repect to end-effector frame
  is_first_ = true;
  insert_first_ = true;

  status_ = 0;
  goal_position_.setZero();
  std::cout<<"START POSITION: "<<pos_init_.transpose()<<std::endl;
}


void TorqueJointSpaceControllerHip::update(const ros::Time& time, const ros::Duration& period) {

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
  q_desired.setZero();

  ros::Duration simulation_time = time - start_time_;
  cur_time_ = time;
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

//---------------------------------code starts from here--------------------------
  Eigen::Vector6d xd;
  Eigen::Matrix<double, 6, 1> f_star_zero;
  Eigen::Matrix<double, 6, 6> lambda;
  Eigen::Matrix<double, 7, 6> J_bar;
  Eigen::Vector3d force_ee; //w.r.t end-effector
  Eigen::Vector3d moment_ee; 
  
  lambda = (jacobian*mass_matrix.inverse()*jacobian.transpose()).inverse();
  J_bar = mass_matrix.inverse()*jacobian.transpose()*lambda;
  f_measured_ = J_bar.transpose()*(tau_measured - gravity); //w.r.t global frame

  force_ee = rotation_M.transpose()*f_measured_.head<3>();
  moment_ee = rotation_M.transpose()*f_measured_.tail<3>();

  xd = jacobian*qd;

  double threshold = -20.0;

  // status_ = 3;

  switch(status_)
  {
    case 0:
      approach(force_ee(assemble_dir_), threshold, position, xd, rotation_M);
      if(checkForceDot(fx_ee_, 8.0))
      {
        status_ = 1;
        std::cout<<"CONTACT IS DETECT!!"<<std::endl;
      } 
      break;
    case 1:
      rasterSearch(position, xd, rotation_M);
      break;
    case 2:
      insert(force_ee(assemble_dir_), threshold, position, xd, rotation_M);
      break;
    case 3:
      verify(force_ee, 10.0, position, xd, rotation_M);
      
  }

  tau_cmd = jacobian.transpose()*f_star_zero_;  
  // tau_cmd.setZero();
//---------------------------------code ends--------------------------
  if (print_rate_trigger_()) {
    // ROS_INFO("--------------------------------------------------");
    // ROS_INFO_STREAM("f_star_zero_ :" << f_star_zero_.transpose());
    // ROS_INFO_STREAM("force_ee :" << force_ee.transpose());
    // ROS_INFO_STREAM("start_position :" << pos_init_.transpose() );
    // ROS_INFO_STREAM("goal_position :" << goal_position_.transpose() );
    // ROS_INFO_STREAM("kcurrent_position :" << position.transpose() );
    // //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    // ROS_INFO_STREAM("assemble_direction :"<< assemble_dir_);
    // ROS_INFO_STREAM("x_curent : "<< position.transpose());
    // ROS_INFO_STREAM("q_desired : "<< q_desired.transpose());
    // ROS_INFO_STREAM("rotation_matrix : "<<"\n"<< rotation_M);
  }
  
  //if(fx_ee_.size() == 0 ) fx_ee_.push_back(force_ee(0));
  Eigen::Vector3d temp = rotation_M*position;

  if(data_save_trigger_())
  {    
    fx_ee_.push_back(force_ee(0));
    fprintf(force_select, "%lf\t\n", fx_ee_.back());
  }

    fprintf(force_moment_ee, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", force_ee(0), force_ee(1), force_ee(2), moment_ee(0), moment_ee(1), moment_ee(2));
    fprintf(vel_ang_ee, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", xd(0), xd(1), xd(2), xd(3), xd(4), xd(5));
    fprintf(pos_ee, "%lf\t %lf\t %lf\t\n", temp(0), temp(1), temp(2));
    // fprintf(joint0_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", gravity(0), gravity(1), gravity(2), gravity(3), gravity(4), gravity(5), gravity(6));

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

}

void TorqueJointSpaceControllerHip::approach(double current_force, double threshold, Eigen::Vector3d x, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d ori)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  
   //Be ee frame!!!
  f_star = straightMoveEE(pos_init_, x, xd, assemble_dir_, 0.005, cur_time_.toSec(), start_time_.toSec(), ori_init_);
  m_star = keepCurrentState(pos_init_, ori_init_, x, ori, xd, 5000, 100).tail<3>();

  
  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;
}

void TorqueJointSpaceControllerHip::rasterSearch(Eigen::Vector3d x, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rot)
{
  int move_dir = 2; // move up

  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;

  if(is_first_)
  {
    pos_init_ = x;
    sgn_ = pow(-1,cnt_);

    if(cnt_ == 1)
    {
      range_ = range_;
      duration_ = duration_;
    }
    else
    {
      range_ = range_*r_;
      duration_ = duration_*r_;
    }

    goal_position_.setZero();    
    goal_position_(move_dir) += sgn_*range_;
    goal_position_ = pos_init_ + ori_init_*goal_position_;

    search_start_time_ = cur_time_;    
    is_first_ = false;

    std::cout<<"sgn_: "<<sgn_<<std::endl;
    std::cout<<"distance: "<<range_<<std::endl;
    std::cout<<"cnt: "<<cnt_<<std::endl;
    std::cout<<"duration: "<<duration_<<std::endl;
    std::cout<<"start_position: "<<pos_init_.transpose()<<std::endl;
    std::cout<<"goal_position_: "<<goal_position_.transpose()<<std::endl;
    std::cout<<"--------------------------------"<<std::endl;

  }

  f_star = oneDofMoveEE(pos_init_, ori_init_, x, xd, cur_time_.toSec(), search_start_time_.toSec(), duration_, sgn_*range_, move_dir);
  m_star = keepCurrentState(pos_init_, ori_init_, x, rot, xd, 5000, 100).tail<3>(); 

  Eigen::Vector3d push;
  push(assemble_dir_) = 5.0;


  Eigen::Vector3d temp = ori_init_*push;
  for(int i = 0; i < 3; i++)
  { 
    if(i == move_dir) f_star(i) += 0.0;
    else  f_star(i) += temp(i);
  }
  
  if(cur_time_.toSec() - search_start_time_.toSec() > duration_)
  {
    is_first_ = true;
    cnt_++;
  }
  
  if(x(assemble_dir_) - pos_init_(assemble_dir_) >= 0.002)
  {
    status_ = 2;
    std::cout<<"Raster search is done"<<std::endl;
  }

  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;

}

void TorqueJointSpaceControllerHip::insert(double current_force, double threshold, Eigen::Vector3d x, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d ori)
{
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Vector3d push;
  
  push << 10.0, 0, 0;

  Eigen::Vector3d start_force;
  Eigen::Vector3d target_force;

  int move_up_dir = 2; //z-axis
  double del;

  if(insert_first_)
  {
    insert_start_time_ = cur_time_;
    // start_force = f_star_zero_.head<3>(); //just expect that it will be the initial force value
    // target_force = keepCurrentState(pos_init_, ori_init_, x, ori, xd, 5000, 100).head<3>();
    
    // del = abs(target_force(move_up_dir) - start_force(move_up_dir));  
    // std::cout<<del<<std::endl;
    // std::cout<<"THE FIRST TICK IN INSERT LOOP"<<std::endl;
    insert_first_ = false;
  }

  // if(del >= 0.01)
  // {
  //   for(int i = 0; i < 1000; i ++)
  //   {
  //     f_star = forceSmoothing(start_force, target_force, i, 1.0);
  //       std::cout<<"try insert"<<std::endl;
  //       std::cout<<"start_force: "<<start_force.transpose()<<std::endl;
  //       std::cout<<"target_force: "<<target_force.transpose()<<std::endl;
  //       std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  //       std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  //   }
  // }
  // else
  // {
  //   f_star = keepCurrentState(pos_init_, ori_init_, x, ori, xd, 5000, 100).head<3>();
  // } 

  f_star = keepCurrentState(pos_init_, ori_init_, x, ori, xd, 5000, 100).head<3>();
  m_star.setZero();

  std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  
  Eigen::Vector3d temp = ori_init_*push;
  f_star(assemble_dir_) = temp(assemble_dir_);


  f_star_zero_.head<3>() = f_star;
  f_star_zero_.tail<3>() = m_star;

  


  std::cout<<"f_star: "<<f_star.transpose()<<std::endl;
  std::cout<<"m_star: "<<m_star.transpose()<<std::endl;
  std::cout<<"push_ee: "<<push.transpose()<<std::endl;
  std::cout<<"push_ee: "<<temp.transpose()<<std::endl;
  std::cout<<"-----------------"<<std::endl;
  
  if( cur_time_.toSec() - insert_start_time_.toSec() >= 3.0)
  {
    status_ = 3;
    std::cout<<"INSERTION IS DONE"<<std::endl;
  }

  // f_star_zero_.setZero();
}

Eigen::Vector3d TorqueJointSpaceControllerHip::forceSmoothing(const Eigen::Vector3d start_force, const Eigen::Vector3d target_force, const int tick, const double duration)
{
  Eigen::Vector3d cmd;

  double current_time = tick / 1000;

  std::cout<<current_time<<std::endl;

  for(size_t i = 0; i < 3; i ++)
  {
    cmd(i) = cubic(current_time, 0.0, duration, start_force(i), target_force(i), 0, 0);
    
    std::cout<<i<<" value: "<<cmd(i)<<std::endl;
  }

  return cmd;
}

void TorqueJointSpaceControllerHip::verify(const Eigen::Vector3d force_ee, const double threshold, Eigen::Vector3d x, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d ori)
{
  double val = abs(threshold);
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  
  // if(check_assembly_(assemble_dir_) == false)
  // {
  //   if(abs(force_ee(assemble_dir_)) >= val) check_assembly_(assemble_dir_) = true;   
  //   f_star = straightMoveEE(pos_init_, x, xd, assemble_dir_, 0.005, cur_time_.toSec(), start_time_.toSec(), ori_init_);
  // } 
  // else
  // {
  //   f_star = keepCurrentState(pos_init_, ori_init_, x, ori, xd, 5000, 100).head<3>();
  //   std::cout<<"COMPLETE!!"<<std::endl;
  // }
  
  // m_star = keepCurrentState(pos_init_, ori_init_, x, ori, xd, 5000, 100).tail<3>();

  // f_star_zero_.head<3>() = f_star;
  // f_star_zero_.tail<3>() = m_star;

  std::cout<<"check direction: "<<check_assembly_.transpose()<<std::endl;
  std::cout<<"check force_ee: "<<force_ee.transpose()<<std::endl;
  std::cout<<"-------------------------"<<std::endl;
}


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerHip,
                       controller_interface::ControllerBase)
