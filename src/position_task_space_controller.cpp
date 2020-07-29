
#include <advanced_robotics_franka_controllers/position_task_space_controller.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include "math_type_define.h"

namespace advanced_robotics_franka_controllers
{

bool PositionTaskSpaceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  //position_data = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/position_data.txt","w");
  //ori_data = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/ori_data.txt","w");
  
	std::vector<std::string> joint_names;
  std::string arm_id;
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

  auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting position joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(position_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }


    std::string file_path = "/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE/";

  save_data_input.open(file_path+"save_data_input.txt");
  save_data_ee.open(file_path+"save_data_ee.txt");



  return true;
}

void PositionTaskSpaceController::starting(const ros::Time& time) {
  start_time_ = time;
  time_ = start_time_;
  elapsed_time_ = ros::Duration(0.0);
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  qd_filtered_.setZero();
  q_desired_ = q_init_;
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  // ori_init_ = transform_init_.linear();  
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

  q_desired_last = q_desired_;

}


void PositionTaskSpaceController::update(const ros::Time& time, const ros::Duration& period) {
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  //franka::RobotState desired_state;
  //desired_state.q = q_desired_;
  //franka::Model desired_model;
  //const std::array<double, 42> &desired_jacobian_array = desired_model.bodyJacobian(franka::Frame::kEndEffector,desired_state);
  const std::array<double, 42> &jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  const std::array<double, 7> &gravity_array = model_handle_->getGravity();
  const std::array<double, 49> &massmatrix_array = model_handle_->getMass();
  const std::array<double, 7> &coriolis_array = model_handle_->getCoriolis();



  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  //Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian_d(desired_jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(massmatrix_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  //if (print_rate_trigger_()) {
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  //}
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> qd(robot_state.dq.data());

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  //Eigen::Affine3d transform_d(Eigen::Matrix4d::Map(desired_model.pose(franka::Frame::kEndEffector,desired_state)));
  Eigen::Vector3d position(transform.translation());
  //Eigen::Vector3d position_d(transform_d.translation());

  Eigen::Vector6d xd_desired;

  Eigen::Vector3d p_goal, p_desired, pd_desired;

  const auto & ori_init_ = transform_init_.linear();
  rotation_ = transform.linear();
  
  const auto & p_init = transform_init_.translation();
  //p_goal = p_init + Eigen::Vector3d::Ones() * 0.1;


    p_goal(0) = p_init(0); //0.85
    p_goal(1) = p_init(1) - 0.3;
    p_goal(2) = p_init(2);// + 0.1;


    double trajectory_time = 10.0; //2.0


  ros::Duration simulation_time = time_ - start_time_;
  Eigen::Matrix<double, 7, 1> q_cmd;
	

  // double cutoff = 40.0; // Hz //20
  // double RC = 1.0 / (cutoff * 2.0 * M_PI);
   double dt = 0.001;
  // double alpha = dt / (RC + dt);
  // for (size_t i = 0; i < 7; i++)
  // {
  //   //qd_filtered_(i) = alpha * robot_state.dq[i] + (1 - alpha) * qd_filtered_(i);
  //   q_filtered_(i) = alpha * robot_state.q[i] + (1 - alpha) * q_filtered_(i);
  // }

  elapsed_time_ += period;

 
  for(int i=0; i<3;i++)
  {
    // p_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
    //                                     p_init(i), p_goal(i), 0, 0);
    // pd_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
    //                                     p_init(i), p_goal(i), 0, 0);
    p_desired(i) = DyrosMath::cubic(elapsed_time_.toSec(), 0, 0 + trajectory_time,
                                        p_init(i), p_goal(i), 0, 0);
    pd_desired(i) = DyrosMath::cubicDot(elapsed_time_.toSec(), 0, 0 + trajectory_time,
                                        p_init(i), p_goal(i), 0, 0);
  }


  double kp, kd, kp_ori;
  // kp = 0.005 / dt;
  // kd = 0.1;
  kp = 1.0;
  kp_ori = 1.0;
  
  //const Eigen::Matrix<double,3,7> & jacobian_velocity =  jacobian_d.block<3,7>(0,0);
  const Eigen::Matrix<double,3,7> & jacobian_velocity =  jacobian.block<3,7>(0,0);
  //Eigen::Vector3d velocity = jacobian_velocity * qd;
  //xd_desired.head<3>() = kp * (p_desired - position_d) + pd_desired;
  xd_desired.head<3>() = kp * (p_desired - position) + pd_desired;
  // Eigen::Vector7d qd_desired = jacobian_velocity.transpose() * 
  //         (jacobian_velocity * jacobian_velocity.transpose() + Eigen::Matrix3d::Identity() * 0.001)
  //         .inverse() * xd_desired.head<3>();



  delphi = DyrosMath::getPhi(rotation_, ori_init_);
  xd_desired.tail<3>() = kp_ori * (-0.5) * delphi;

  Eigen::Vector7d qd_desired = jacobian.transpose() * 
          (jacobian * jacobian.transpose() + Eigen::Matrix6d::Identity() * 0.001)
          .inverse() * xd_desired;


 // q_desired_ += qd_desired * dt;
  q_desired_ = q_desired_ + qd_desired * period.toSec();



    for(int i=0; i<7;i++)
  {
    q_desired_(i) = franka::lowpassFilter(0.001, q_desired_(i), q_desired_last(i), 50.0);
    //q_desired(i) = 0.9 * q_desired_last(i) + 0.1 * q_desired(i);
  }

  
  //q_desired_ = q + qd_desired * dt;
  //q_desired_ = 0.99 * q_desired_ + 0.01 * q + qd_desired * dt;

  // Eigen::Vector7d i_7;
  // for (size_t i = 0; i < 7; i++)
  // {
  //  i_7(i) = 1;
  // }
  // q_desired_ = q + i_7 * 0.002;

  //q_desired_ = 0.99 * q_desired_ + 0.01 * q;
  //q_desired_ = q_filtered_;

  //q_desired_ = q_init_;
  // if (print_rate_trigger_()) {
  // q_desired_ = q;
  // }
  // q_cmd = q_desired_;

  // std::cout << "xd_desired" << std::endl <<
  //         xd_desired.transpose() << std::endl <<
  //         "qd_desired" << std::endl <<
  //         qd_desired.transpose() << std::endl <<
  //         "jacobian_velocity" << std::endl << 
  //         jacobian_velocity << std::endl <<
  //         "q_cmd" << std::endl <<
  //         q_cmd << std::endl;

  // q_cmd = q_init_;

//   //with ori
//   delphi = DyrosMath::getPhi(rotation_, ori_init_);

// //   //////
//   double kp, kd, kp_ori;
//   kp = 0.01;
//   kd = 0.1;
//   kp_ori = 0.01;

//   const Eigen::Matrix<double,3,7> & jacobian_velocity =  jacobian.block<3,7>(0,0);
//   Eigen::Vector3d velocity = jacobian_velocity * qd;
//   xd_desired.head<3>() = pd_desired + kp * (p_desired - position);
// xd_desired.tail<3>() = kp_ori * (-1.0) * delphi;

//   Eigen::Vector7d qd_desired = jacobian.transpose() * 
//           (jacobian * jacobian.transpose() + Eigen::Matrix6d::Identity() * 0.001)
//           .inverse() * xd_desired;
// //  q_desired_ = q_desired_ + qd_desired * period.toSec();
//   q_desired_ = q_desired_ + qd_desired /1000;
  
//   //q_desired_ += qd_desired * 0.001;
//   //q_desired_ = q + qd_desired * period.toSec();
//   //q_cmd = q_desired_;


  
  if (print_rate_trigger_()) {
    ROS_INFO("--------------------------------------------------");
    //ROS_INFO_STREAM("tau :" << q_cmd.transpose());
    //ROS_INFO_STREAM("time :"<< simulation_time);
    ROS_INFO_STREAM("q_curent : "<< q.transpose());
    ROS_INFO_STREAM("q_desired : "<< q_desired_.transpose());


  }
  //time_ = time_ + ros::Duration(dt);
  
  //Eigen::VectorXd q_error;
  //q_error = q_desired_ - q;
  //fprintf(position_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t \n", q_error(0) ,  q_error(1) , q_error(2), q_error(3), q_error(4), q_error(5),q_error(6));
 
  Eigen::VectorXd p_error;
  p_error = p_desired - position;
  //fprintf(position_data, "%lf\t %lf\t %lf\t \n", p_error(0) ,  p_error(1) , p_error(2));
  //fprintf(ori_data, "%lf\t %lf\t %lf\t \n", delphi(0) ,  delphi(1) , delphi(2));

  for (size_t i = 0; i < 7; ++i) {
//    joint_handles_[i].setCommand(q_cmd(i));
    joint_handles_[i].setCommand(q_desired_(i));
  }

  q_desired_last = q_desired_;

  save_data_input.save(q_desired_.transpose());
  save_data_ee.save(position.transpose());

}


} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::PositionTaskSpaceController,
                       controller_interface::ControllerBase)
