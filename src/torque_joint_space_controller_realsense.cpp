#include <advanced_robotics_franka_controllers/torque_joint_space_controller_realsense.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <iostream>

#include <franka/robot_state.h>
#include <franka/model.h>

#include "math_type_define.h"

namespace advanced_robotics_franka_controllers
{

bool TorqueJointSpaceControllerRealsense::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
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

  target_3d_points_sub_ = node_handle.subscribe("/target_3d_points_topic", 1, &TorqueJointSpaceControllerRealsense::targePointCallback,this);
  return true;
}

void TorqueJointSpaceControllerRealsense::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  pos_init_ = transform_init_.translation();	
  ori_init_ = transform_init_.rotation();
  
  

  
  cam_p_obj_.setZero();
  ee_p_cam_.setZero();
  uni_p_obj_.Identity();
  uni_p_ee_ = transform_init_.translation();

  cam_r_obj_.setZero();
  ee_r_cam_ << -1, 0, 0, 0, -1, 0, 0, 0, 1;
  uni_r_obj_.Identity();
  uni_r_ee_ = transform_init_.rotation();

  cam_T_obj_.setZero();
  ee_T_cam_.setZero(); 
  uni_T_obj_.setZero();
  uni_T_ee_.setZero();

  std::ifstream target_object;
  target_object.open("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LHS/target_object.txt");
  target_object >> uni_p_obj_(0) >> uni_p_obj_(1) >> uni_p_obj_(2);
  target_object.close();
}


void TorqueJointSpaceControllerRealsense::update(const ros::Time& time, const ros::Duration& period) {

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

  for(size_t i = 0; i < obj_x_.size(); i++)
  {
    uni_r_ee_ = rotation_M;
    cam_r_obj_ = ee_r_cam_.transpose()*uni_r_ee_.transpose();
    
    cam_p_obj_(0) = obj_x_.at(i);
    cam_p_obj_(1) = obj_y_.at(i);
    cam_p_obj_(2) = obj_z_.at(i);

    uni_p_ee_ = position;

    cam_T_obj_ <<cam_r_obj_.row(0), cam_p_obj_(0), cam_r_obj_.row(1), cam_p_obj_(1), cam_r_obj_.row(2), cam_p_obj_(2), 0, 0, 0, 1;
    uni_T_obj_ <<uni_r_obj_.row(0), uni_p_obj_(0), uni_r_obj_.row(1), uni_p_obj_(1), uni_r_obj_.row(2), uni_p_obj_(2), 0, 0, 0, 1;
    uni_T_ee_ <<uni_r_ee_.row(0), position(0), uni_r_ee_.row(1), position(1), uni_r_ee_.row(2), position(2), 0, 0, 0, 1;
    
    Eigen::Matrix4d uni_T_ee_inv;
    Eigen::Matrix4d cam_T_obj_inv;

    uni_T_ee_inv.block<3,3>(0,0) = uni_r_ee_.transpose();
    uni_T_ee_inv.col(3) << -uni_r_ee_.col(0).transpose()*uni_p_ee_, -uni_r_ee_.col(1).transpose()*uni_p_ee_, -uni_r_ee_.col(2).transpose()*uni_p_ee_, 1;

    cam_T_obj_inv.block<3,3>(0,0) = cam_r_obj_.transpose();
    cam_T_obj_inv.col(3) << -cam_r_obj_.col(0).transpose()*cam_p_obj_, -cam_r_obj_.col(1).transpose()*cam_p_obj_, -cam_r_obj_.col(2).transpose()*cam_p_obj_, 1;

    ee_T_cam_ = uni_T_ee_inv*uni_T_obj_*cam_T_obj_inv;

    if (print_rate_trigger_()) {
    ROS_INFO("--------------------------------------------------");
    ROS_INFO_STREAM("cam_p_obj_ : "<< cam_p_obj_.transpose());
    ROS_INFO_STREAM("cam_T_obj_inv : "<<"\n"<< cam_T_obj_inv);
    ROS_INFO_STREAM("uni_T_ee_ : "<<"\n"<< uni_T_ee_);
  
    } 
  }
  


    
//tau_cmd =  mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd));

////  tau_cmd =  mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd));// + coriolis;
  //tau_cmd = -gravity + graivity_mod;
  tau_cmd.setZero();

  if (print_rate_trigger_()) {
    // ROS_INFO("--------------------------------------------------");
    // ROS_INFO_STREAM("tau :" << gravity.transpose());
    //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    // ROS_INFO_STREAM("time :"<< simulation_time);
    // ROS_INFO_STREAM("q_curent : "<< q.transpose());
    // ROS_INFO_STREAM("uni_p_obj_ : "<< uni_p_obj_.transpose());
    // ROS_INFO_STREAM("cam_T_obj_ : "<<"\n"<< cam_T_obj_);
    // ROS_INFO_STREAM("uni_T_ee_ : "<<"\n"<< uni_T_ee_);
    ROS_INFO_STREAM("no vaild data");

  } 

//  fprintf(joint0_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", time.toSec(), q_desired(0), q(0), qd(0), tau_cmd(0), tau_J_d(0), tau_measured(0), gravity);
    
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

}

void TorqueJointSpaceControllerRealsense::targePointCallback(const geometry_msgs::PoseArrayPtr &msg)
{
  for(auto &pose : msg -> poses)
  {
    obj_x_.push_back(pose.position.x);
    obj_y_.push_back(pose.position.y);  
    obj_z_.push_back(pose.position.z);
  }
}

} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerRealsense,
                       controller_interface::ControllerBase)
