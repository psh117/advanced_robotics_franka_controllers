#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/trigger_rate.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/franka_gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>

#include "math_type_define.h"

#include <fstream>
#include <iostream>
#define SIZE 500

namespace advanced_robotics_franka_controllers {

class TorqueJointSpaceControllerPlace : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void initConfig(const ros::Time& time, const Eigen::Vector3d position, const Eigen::Matrix3d rotation_M, const Eigen::Matrix<double, 6, 1> x_dot_);
  void approach(const ros::Time& time, const Eigen::Vector3d position, const Eigen::Matrix3d rotation_M, const Eigen::Matrix<double, 6, 1> x_dot_);
  void alignAxis(const ros::Time& time, const Eigen::Vector3d position, const Eigen::Matrix3d rotation_M, const Eigen::Matrix<double, 6, 1> x_dot_, const double duration);
  void keepState(const ros::Time& time, const Eigen::Vector3d position, const Eigen::Matrix3d rotation_M, const Eigen::Matrix<double, 6, 1> x_dot_);
  void getInitialFT(const int index);
  void getDirectionVector(const Eigen::Vector3d position, const Eigen::Matrix3d rotation);
  void clearDirectionVector();
  void getMoment(const Eigen::Matrix<double, 6, 1> f, const Eigen::Matrix<double, 6, 1> f_ee);
  void clearMoment();
  void saveReasult();
  void forceSmoothing(const Eigen::Vector3d goal_f, Eigen::Vector3d cur_f, const ros::Time& cur_time, const double duration);

 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;


  //std::unique_ptr<franka_gripper::grasp> gripper_;
  

  ros::Time start_time_;
  
  franka_hw::TriggerRate print_rate_trigger_{10}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;
  Eigen::Vector3d pos_init_;
  Eigen::Matrix<double, 3, 3> ori_init_;


  
  FILE *save_data_x;
  FILE *save_data_x2;
  FILE *save_cmd;
  FILE *save_dir;
  FILE *save_result;

  Eigen::Vector3d target_x_;
  Eigen::Vector3d x_desired_;
  Eigen::Vector3d xdot_desired_;
  
  Eigen::Matrix<double, 3, 3> K_p;
	Eigen::Matrix<double, 3, 3> K_v;
  Eigen::Matrix<double, 3, 3> K_p_ori;

  Eigen::Vector3d f_star_;
	Eigen::Vector3d m_star_;
	Eigen::Matrix<double, 6, 1> f_star_zero_;

  
  Eigen::Matrix<double, 3, 3> target_rotation_;


  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d euler_angle_;

  Eigen::Matrix<double, 6, 1> f_sensing_;
  Eigen::Matrix<double, 6, 1> f_sensing_ee_;
  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_grasp_{"/franka_gripper/grasp", true};  
  franka_gripper::GraspGoal goal;

  int check_gripper;

  Eigen::Vector3d init_origin_;
  Eigen::Vector3d approach_origin_;
  Eigen::Vector3d keep_state_origin_;
  Eigen::Vector3d revolve_origin_;

  Eigen::Matrix3d initial_rotation_M;

  Eigen::Matrix3d first_rotation_M;
  Eigen::Matrix3d last_rotation_M;

  Eigen::Vector3d rotation_dir_;
  
  double init_config_start_time_;
  double approach_start_time_;
  double keep_state_start_time_;
  double revolve_start_time_;
  double smoothing_start_time_;
  
  bool detect_contact_;
  bool rotation_done_;
  bool is_config_first_;
  bool is_approach_first_;
  bool is_revolve_first_;
  bool is_keep_state_first_;

  double initial_force_;
  double force_sum_;
  double final_force_;
  Eigen::Vector3d initial_moment_;
  Eigen::Vector3d moment_sum_;

  Eigen::Vector3d ee_z_axis_;
  Eigen::Vector3d normal_vector_;
  Eigen::Vector3d rotation_axis_;
  Eigen::Vector3d p1_;
  Eigen::Vector3d p2_;
  Eigen::Vector3d p3_;
  double angle_difference_;


  int index_;
  int contact_count_;
  int contact_points_;
  int revolve_direction_;

  std::vector<double> nx_;
  std::vector<double> ny_;
  std::vector<double> nz_;
  std::vector<double> ex_;
  std::vector<double> ey_;
  std::vector<double> ez_;
  std::vector<double> p_;

  std::vector<double> mx_;
  std::vector<double> my_;
  std::vector<double> mz_;
  std::vector<double> mx_ee_;
  std::vector<double> my_ee_;
  std::vector<double> mz_ee_;
  std::vector<double> t_;
};

static Eigen::Matrix<double, 6, 1> straightApproach(const Eigen::Vector3d origin,
    const Eigen::Matrix3d initial_rotation_M,
    const Eigen::Matrix3d rotation_M,
    const Eigen::Vector3d current_position,
    const Eigen::Matrix<double, 6, 1> current_velocity,
    const double current_time,
    const double init_time)
{
  double desent_speed = -0.005; // 5cm/s

  Eigen::Vector3d desired_position;
  Eigen::Vector3d desired_linear_velocity;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d f_star;
  Eigen::Vector3d m_star;
  Eigen::Matrix<double, 6, 1> f_star_zero;
  Eigen::Matrix3d K_p; 
  Eigen::Matrix3d K_v;
  
  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 100, 0, 0, 0, 100, 0, 0, 0, 100;

  desired_position.head<2>() = origin.head<2>();
  desired_position(2) = origin(2) + desent_speed*(current_time - init_time);
  desired_linear_velocity << 0, 0, desent_speed;
  
  delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, initial_rotation_M);
  f_star = K_p * (desired_position - current_position) + K_v * ( desired_linear_velocity- current_velocity.head<3>());  
  m_star = (1.0) * 200.0* delphi_delta+ 5.0*(-current_velocity.tail<3>()) ;
  
  f_star_zero.head<3>() = f_star;
  f_star_zero.tail<3>() = m_star;

  return f_star_zero;    
}

static Eigen::Vector3d keepCurrentPosition(const Eigen::Vector3d origin,
        const Eigen::Vector3d current_position,
        const Eigen::Matrix<double, 6, 1> current_velocity,
        const double kp = 5000,
        const double kv = 200)
{
  Eigen::Vector3d x_d;
  Eigen::Vector3d f_star;
  Eigen::Matrix3d K_p; 
  Eigen::Matrix3d K_v;

  K_p << kp, 0, 0, 0, kp, 0, 0, 0, kp;
  K_v << kv, 0, 0, 0, kv, 0, 0, 0, kv;

  x_d = origin;

  f_star = K_p * (x_d - current_position);// + K_v * (- current_velocity.head<3>());  
  
  return f_star;
}  

static Eigen::Vector3d keepCurrentOrientation(const Eigen::Matrix3d initial_rotation_M,
  const Eigen::Matrix3d rotation_M,
  const Eigen::Matrix<double, 6, 1> x_dot)
{
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;
 
  delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, initial_rotation_M);
  m_star = (1.0) * 200.0* delphi_delta;//+ 5.0*(-x_dot.tail<3>());  

  return m_star;
}

static Eigen::Vector3d keepOrientationPerpenticularOnlyXY(const Eigen::Matrix3d initial_rotation_M,
  const Eigen::Matrix3d rotation_M,
  const Eigen::Matrix<double, 6, 1> current_velocity,
  const double duration,
  const double current_time,
  const double init_time)
{
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;
  Eigen::Vector3d euler_angle;

  double val;
  double e;

  double roll, alpha;
  double pitch, beta;
  double yaw, gamma;

  double min;
  int index;

  euler_angle = DyrosMath::rot2Euler(initial_rotation_M);
  roll = euler_angle(0);
  pitch = euler_angle(1);
  yaw = euler_angle(2);

  val = initial_rotation_M(2, 2);
  e = 1.0 - fabs(val);

  if (val > 0) //upward
  {
    roll = 0;
    pitch = 0;
  }
  
  else //downward
  {
    if (roll > 0) roll = 180 * DEG2RAD;
    else roll = -180 * DEG2RAD;

    pitch = 0;
  }

  alpha = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(0), roll, 0, 0);
  beta = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(1), pitch, 0, 0);
  gamma = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(2), yaw, 0, 0);

  target_rotation_M = DyrosMath::rotateWithZ(gamma) * DyrosMath::rotateWithY(beta) * DyrosMath::rotateWithX(alpha);

  delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 250.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

  return m_star;
}

static Eigen::Vector3d vectorNormalization(const Eigen::Vector3d v)
{
  Eigen::Vector3d n;

  n = v/sqrt(pow(v(0),2) + pow(v(1),2) + pow(v(2),2));

  return n;
}

static Eigen::Vector3d computeNomalVector(const Eigen::Vector3d p1,
  const Eigen::Vector3d p2,
  const Eigen::Vector3d p3)
{
  Eigen::Vector3d r1;
  Eigen::Vector3d r2;
  Eigen::Vector3d r3;
  Eigen::Vector3d result;

  r1 = p1 - p2;
  r2 = p1 - p3;

  r3 = r1.cross(r2);
  r3 = r3/sqrt(pow(r3(0),2) + pow(r3(1),2) + pow(r3(2),2));

  result = r3;
  return result;
}

static Eigen::Vector3d computeRotationAxis(const Eigen::Vector3d EE_z_axis, //compare nomal vector of a plane and end-effector
  Eigen::Vector3d normal_vector)
{
  Eigen::Vector3d result;

  if(EE_z_axis(2)*normal_vector(2) < 0) normal_vector = -normal_vector;

  result = EE_z_axis.cross(normal_vector);  

  return result;
}

static double computeAngleDifference(const Eigen::Vector3d EE_z_axis,
  const Eigen::Vector3d normal_vector)
{
  double result;
  double dot;
  double m1;
  double m2;

  dot = EE_z_axis.dot(normal_vector);
  m1 = sqrt(pow(EE_z_axis(0),2) + pow(EE_z_axis(1),2) + pow(EE_z_axis(2),2));
  m2 = sqrt(pow(normal_vector(0),2) + pow(normal_vector(1),2) + pow(normal_vector(2),2));

  result = dot/(m1*m2); //cos(x) = (a*b)/(|a|*|b|)
  result = acos(result);

  return result;
}

static Eigen::Vector3d rotateWithGivenAxis(const Eigen::Vector3d rotation_axis,
  const Eigen::Matrix3d initial_rotation_M,
  const Eigen::Matrix3d rotation_M,
  const Eigen::Matrix<double, 6, 1> current_velocity,
  const double angular_vel,
  const double goal,
  const double current_time,
  const double init_time)
{
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d m_star;
  Eigen::Vector3d delphi_delta;

  double ux = rotation_axis(0);
  double uy = rotation_axis(1);
  double uz = rotation_axis(2);

  double theta;

  double r11, r12, r13;
  double r21, r22, r23;
  double r31, r32, r33;
  theta = 0.0;
  
  theta = angular_vel*(current_time - init_time);

  if(fabs(theta) >= fabs(goal)) theta = goal;

  r11 = cos(theta) + pow(ux,2)*(1-cos(theta));
  r12 = ux*uy*(1-cos(theta)) - uz*sin(theta);
  r13 = ux*uz*(1-cos(theta) )+ uy*sin(theta);

  r21 = ux*uy*(1-cos(theta)) + uz*sin(theta);
  r22 = cos(theta) + pow(uy,2)*(1-cos(theta));
  r23 = uy*uz*(1-cos(theta)) - ux*sin(theta);

  r31 = ux*uz*(1-cos(theta)) - uy*sin(theta);
  r32 = uz*uy*(1-cos(theta)) + ux*sin(theta);
  r33 = cos(theta) + pow(uz,2)*(1-cos(theta));

  target_rotation_M << r11, r12, r13, r21, r22, r23, r31, r32, r33;
  target_rotation_M = target_rotation_M * initial_rotation_M;

  delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

  // std::cout<<"-------------------------"<<std::endl;
  // std::cout<<"goal: "<<goal*180/M_PI<<std::endl;
  // std::cout<<"theta: "<<theta*180/M_PI<<std::endl;
  // std::cout<<"time: "<<current_time-init_time<<std::endl;
  return m_star;

}



}  // namespace advanced_robotics_franka_controllers
