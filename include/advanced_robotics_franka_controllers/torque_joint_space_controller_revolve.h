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

class TorqueJointSpaceControllerRevolve : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

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

  Eigen::Vector3d f_star_;
	Eigen::Vector3d m_star_;
	Eigen::Matrix<double, 6, 1> f_star_zero_;

  Eigen::Matrix<double, 6, 1> f_sensing_;
  Eigen::Matrix<double, 6, 1> f_sensing_ee_;
  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_grasp_{"/franka_gripper/grasp", true};  
  franka_gripper::GraspGoal goal;

  int check_gripper;

  bool is_first_;
  bool rotate_is_done_;
  double f_sum_;
  int cnt_;

  int dir_;
};

static Eigen::Vector3d keepCurrentPosition(const Eigen::Vector3d origin,
        const Eigen::Vector3d current_position,
        const Eigen::Matrix<double, 6, 1> current_velocity)
{
  Eigen::Vector3d x_d;
  Eigen::Vector3d f_star;
  Eigen::Matrix3d K_p; 
  Eigen::Matrix3d K_v;

  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

  x_d = origin;

  f_star = K_p * (x_d - current_position) + K_v * (- current_velocity.head<3>());  
  
  return f_star;
}  

static Eigen::Vector3d keepCurrentOrientation(const Eigen::Matrix3d init_rot,
      const Eigen::Matrix3d current_rot,
      const Eigen::Matrix<double, 6, 1> current_velocity)
{
  Eigen::Vector3d m_star;
  Eigen::Vector3d delphi_delta;

  delphi_delta = -0.5 * DyrosMath::getPhi(current_rot, init_rot);

  m_star = (1.0) * 200.0* delphi_delta+ 5.0*(-current_velocity.tail<3>());

  return m_star;    
}

static Eigen::Vector3d keepOrientationPerpenticular(const Eigen::Matrix3d initial_rotation_M,
  const Eigen::Matrix3d rotation_M,
  const Eigen::Matrix<double, 6, 1> current_velocity,
  const double duration,
  const double current_time,
  const double init_time)
{
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d delphi_delta;
  Eigen::Vector3d m_star;
  Eigen::Vector5d angle_set_45;
  Eigen::Vector5d angle_set_error;
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

  // angle_set_45 << -135, -45, 45, 135;
  angle_set_45 << -180.0, -90.0, 0.0, 90.0, 180.0;
  angle_set_45 = angle_set_45 * DEG2RAD;

  if (val > 0 && e <= 0.01 * DEG2RAD) //upward
  {
    roll = 0;
    pitch = 0;

    for (size_t i = 0; i < 5; i++)
    {
      angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(2));
    }

    for (size_t i = 0; i < 5; i++)
    {
      if (angle_set_error(i) == angle_set_error.minCoeff()) index = i;
    }


    yaw = angle_set_45(index);

  }
  else if (val < 0 && e <= 0.01 * DEG2RAD) //downward
  {
    if (roll > 0) roll = 180 * DEG2RAD;
    else roll = -180 * DEG2RAD;

    pitch = 0;

    for (size_t i = 0; i < 5; i++)
    {
      angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(2));
    }
    for (size_t i = 0; i < 5; i++)
    {
      if (angle_set_error(i) == angle_set_error.minCoeff()) index = i;
    }

    yaw = angle_set_45(index);
  }

  else //on xy plane
  {
    roll = euler_angle(0);
    yaw = euler_angle(2);

    for (size_t i = 0; i < 5; i++)
    {
      angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(1));
    }

    for (size_t i = 0; i < 5; i++)
    {
      if (angle_set_error(i) == angle_set_error.minCoeff()) index = i;
    }

    pitch = angle_set_45(index);
  }

  alpha = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(0), roll, 0, 0);
  beta = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(1), pitch, 0, 0);
  gamma = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(2), yaw, 0, 0);

  target_rotation_M = DyrosMath::rotateWithZ(gamma) * DyrosMath::rotateWithY(beta) * DyrosMath::rotateWithX(alpha);

  delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 250.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

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

static Eigen::Vector3d rotateWithLocalAxis(const Eigen::Matrix3d initial_rotation_M,
  const Eigen::Matrix3d rotation_M,
  const Eigen::Matrix<double, 6, 1> current_velocity,
  const double goal,
  const double current_time,
  const double init_time,
  const int dir)
{
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d m_star;
  Eigen::Vector3d delphi_delta;

  double theta;
  double duration = 5.0;
  
  theta = 0.0;

  theta = DyrosMath::cubic(current_time, init_time, init_time + duration, 0.0, goal, 0, 0);
  
  if(dir == 0) target_rotation_M = DyrosMath::rotateWithX(theta);
  if(dir == 1) target_rotation_M = DyrosMath::rotateWithY(theta);
  if(dir == 2) target_rotation_M = DyrosMath::rotateWithZ(theta);


  target_rotation_M = initial_rotation_M * target_rotation_M;

  delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);

  m_star = (1.0) * 200.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

  if(current_time - init_time >= duration)
  {
    std::cout<<"rotation is done"<<std::endl;
  }
  return m_star;

}


static Eigen::Vector3d oneDofMoveEE(const Eigen::Vector3d origin,
  const Eigen::Matrix3d init_rot,
  const Eigen::Vector3d current_position,
  const Eigen::Matrix<double, 6, 1> current_velocity,
  const double current_time,
  const double init_time,
  const double duration,
  const double target_distance, // + means go forward, - mean go backward
  const int direction) // 0 -> x_ee, 1 -> y_ee, 2 -> z_ee //DESIRED DIRECTION W.R.T END EFFECTOR!!
{
  Eigen::Vector3d goal_position;
  Eigen::Vector3d cmd_position;
  Eigen::Vector3d f_star;
  Eigen::Matrix3d K_p; 
  Eigen::Matrix3d K_v;
  double theta; //atfer projection the init_rot onto the global frame, the theta means yaw angle difference.
  
  K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
  K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;
  

  // start from EE
  for(int i = 0; i < 3; i++)
  {
    if( i == direction) goal_position(i) = target_distance;
    else goal_position(i) = 0.0;
  }

  goal_position = origin + init_rot*goal_position;

  for(int i = 0; i < 3; i++)
  {
    cmd_position(i) = DyrosMath::cubic(current_time, init_time, init_time + duration, origin(i), goal_position(i), 0, 0);
  }  

  f_star = K_p * (cmd_position - current_position) + K_v * (- current_velocity.head<3>());  
  
  return f_star;    
}

static Eigen::Vector3d straightMoveEE(const Eigen::Vector3d origin,
    const Eigen::Matrix3d init_rot,
    const Eigen::Vector3d current_position,
    const Eigen::Matrix<double, 6, 1> current_velocity,
    const int dir,
    const double speed,
    const double current_time,
    const double init_time)
{
    // double desent_speed = -0.02; //-0.005; // 5cm/s

    Eigen::Vector3d goal_position;
    Eigen::Vector3d desired_linear_velocity;
    Eigen::Vector3d f_star;
    Eigen::Matrix3d K_p; 
    Eigen::Matrix3d K_v;
    
    K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
    K_v << 100, 0, 0, 0, 100, 0, 0, 0, 100;

    
    goal_position.setZero();
    goal_position(dir) = speed*(current_time - init_time);

    goal_position = origin + init_rot*goal_position;
    
    desired_linear_velocity.setZero();
    desired_linear_velocity(dir) = speed;
            
    
    f_star = K_p * (goal_position - current_position) + K_v * ( desired_linear_velocity- current_velocity.head<3>());  
    
    
    return f_star;
}
   
}  // namespace advanced_robotics_franka_controllers
