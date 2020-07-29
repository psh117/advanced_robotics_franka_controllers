
#include <advanced_robotics_franka_controllers/position_joint_space_controller_joint_test.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include "math_type_define.h"

namespace advanced_robotics_franka_controllers
{

bool PositionJointSpaceControllerJointTest::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{

  //hqp_joint_vel = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/hqp_joint_acc.txt","w");
  //hqp_joint_pos = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/hqp_joint_pos.txt","w");
  //hqp_joint_tor = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/hqp_joint_pos.txt","w");

  joint0_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint0_data.txt","w");  
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

  joint_data.open(file_path+"joint_data.txt");
  save_data1.open(file_path+"save_data1.txt");

  return true;
}

void PositionJointSpaceControllerJointTest::starting(const ros::Time& time) {
  start_time_ = time;
  elapsed_time_ = ros::Duration(0.0);
	//elapsed_time_ = time;

  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
    // q_goal_(i) = q_init_(i);
  }

//  q_goal_ << 0, -M_PI/6, 0, -2*M_PI/3, 0, M_PI/2, M_PI/4;
//  q_goal_ << 1.585978991,	-0.715794858,	-1.745485357,	-1.817219141,	-0.501131713,	3.441684876,	0.567163908;
  // q_goal_ << M_PI/6, M_PI/6, M_PI/6, M_PI/6, M_PI/6, M_PI/6, M_PI/6;

  // q_goal_(0) = 0.042322066;
  // q_goal_(1) = -0.963603021;
  // q_goal_(2) = 0.054226183;
  // q_goal_(3) = -2.644484580;
  // q_goal_(4) = 0.047856795;
  // q_goal_(5) = 1.696981126;
  // q_goal_(6) = -0.813655319;

  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  
}


void PositionJointSpaceControllerJointTest::update(const ros::Time& time, const ros::Duration& period) {
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

  //Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d_data(robot_state.tau_J_d_data.data());

  Eigen::Matrix<double , 6, 7> jacobian_euler;
  Eigen::Matrix<double , 12, 7> jacobian_dc;
  Eigen::Matrix<double , 7, 1> q_goal;
  Eigen::Matrix<double , 7, 1> q_desired;
  Eigen::Matrix<double , 7, 1> qd_desired;
  Eigen::Matrix<double , 12, 1> x_goal; 
  Eigen::Matrix<double , 12, 1> x_desired;
  Eigen::Matrix<double , 12, 1> x_current;
  
  //q_goal.setZero();
  q_goal << 0,0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
  q_desired.setZero();
	
  ros::Duration simulation_time = time - start_time_;
  Eigen::Matrix<double, 7, 1> tau_cmd;
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation());

  elapsed_time_ += period;

  double trajectory_time = 20.0;

  // for(int i = 0; i < 7; i++)
  // {
  //   q_goal(i) = q_init_(i);
  //   if( i == 5) q_goal(i) = q_init_(i)+M_PI/6;
  // }

  for(int i=0; i<7;i++)
  {
//     q_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
//                                         q_init_(i), q_goal_(i), 0, 0);
    //  qd_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
    //                                      q_init_(i), q_goal_(i), 0, 0);

    q_desired(i) = DyrosMath::cubic(elapsed_time_.toSec(), 0, 0 + trajectory_time, q_init_(i), q_goal(i), 0, 0);


    //q_desired(i) = q_goal_(i);
  }
//  q_desired = q_desired + qd_desired / 1000;

  double kp, kv;
  kp = 1500;
  kv = 10;
  //tau_cmd = mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd)) + coriolis;

  //q_desired(5) = q_goal_(5) + 0.3 * sin((time.toSec() - start_time_.toSec()) * 2 * M_PI / 5 - M_PI/2) + 0.3;
  double amplitude = 45.0*3.1415926535/180.0;
  double frequency = 0.0375;
// q_desired(0) = q_goal_(0) + amplitude*(1.0-cos(2.0*3.1415926535*frequency*(time.toSec() - start_time_.toSec())));

  if (print_rate_trigger_()) {
    ROS_INFO("--------------------------------------------------");
    //ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    //ROS_INFO_STREAM("time :"<< simulation_time);
    ROS_INFO_STREAM("q_curent : "<< q.transpose());
    ROS_INFO_STREAM("q_desired : "<< q_desired.transpose());
    // ROS_INFO_STREAM("tau_J_d : "<< tau_J_d.transpose());
    // ROS_INFO_STREAM("time : "<< time.toSec());
    // ROS_INFO_STREAM("time : "<< elapsed_time_.toSec());
  }

  
//fprintf(joint0_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", q(0), q(1), q(2), q(3), q(4), q(5), q(6));
  fprintf(joint_cmd, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", q_desired(0), q_desired(1), q_desired(2), q_desired(3), q_desired(4), q_desired(5), q_desired(6));
  fprintf(joint_real, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", q(0), q(1), q(2), q(3), q(4), q(5), q(6));
  for (size_t i = 0; i < 7; ++i) {
    //joint_handles_[i].setCommand(tau_cmd(i));
    joint_handles_[i].setCommand(q_desired(i));
  }

  //save_data1.save(q_desired(6));
  //joint_data.save(q(6));

}

} // namespace advanced_robotics_franka_controllers



PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::PositionJointSpaceControllerJointTest,
                       controller_interface::ControllerBase)
