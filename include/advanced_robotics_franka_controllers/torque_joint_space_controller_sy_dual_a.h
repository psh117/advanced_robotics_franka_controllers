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

namespace advanced_robotics_franka_controllers {

class TorqueJointSpaceControllerSyDualA : public controller_interface::MultiInterfaceController<
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
  ros::Time rotation_start_time_;
  ros::Time approach_start_time_;
  ros::Time spiral_start_time_;
  ros::Time insert_start_time_;

  franka_hw::TriggerRate print_rate_trigger_{10}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;
  Eigen::Vector3d pos_init_;
  Eigen::Matrix<double, 3, 3> ori_init_;
  Eigen::Matrix<double , 12, 1> x_temp_;
  Eigen::Matrix<double, 3, 3> ori_first_state_, ori_return_state_;


  FILE *joint0_data;
  FILE *save_data_x;
  FILE *save_data_x2;
  FILE *save_data_x3;

  FILE *save_cmd;
  FILE *save_fm;

  Eigen::Vector3d target_x_;
  Eigen::Vector3d x_desired_;
  Eigen::Vector3d pos_first_state_, pos_return_state_;
  Eigen::Vector3d xdot_desired_;
  Eigen::Vector3d spiral_origin_;
  Eigen::Vector3d spiral_done_pos_;
  Eigen::Vector3d spiral_fail_check_step_;
  
  Eigen::Matrix<double, 3, 3> K_p;
	Eigen::Matrix<double, 3, 3> K_v;
  Eigen::Matrix<double, 3, 3> K_p_ori;

  Eigen::Vector3d f_star_;
	Eigen::Vector3d m_star_;
	Eigen::Matrix<double, 6, 1> f_star_zero_;

  Eigen::Matrix<double, 6, 1> target_f;

  Eigen::Matrix<double, 3, 7> jacobian_pos_;

  Eigen::Matrix<double, 3, 3> target_rotation_;
  Eigen::Matrix<double, 3, 3> tilt_rotation_;

  double tilt_angle_z_;
  double rotation_init_;
  double ori_theta_z_;
  double rotation_duration_;
  int ori_change_direction;
  int ori_check_time;

  double contact_force_;
  double descent_speed_;

  double spiral_linear_velocity_;
  double spiral_pitch_;
  double spiral_duration_;
  double spiral_depth_;
  double spiral_force_limit_;
  double spiral_force_;

  double curved_approach_distance_;
  double curved_approach_angle_;
  double curved_approach_lin_vel_;
  double curved_approach_force_;

  Eigen::Matrix<double, 3, 3> rotation_z_theta_;

  Eigen::Vector3d delphi_delta;
  
  Eigen::Matrix<double, 6, 1> f_sensing;
  
  Eigen::Vector3d stop_x;
  
  Eigen::Vector3d pivot_;

  //Eigen::Vector3d x_last_desired_;
  Eigen::Vector3d x_last_desired_1_;
  Eigen::Vector3d x_last_desired_2_;
  
  ros::Duration finish_time;
  ros::Duration spiral_time;
  ros::Duration approach_time;
  ros::Duration insert_time;

  int check_stop;
  // actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_ac_
  // {"/franka_gripper/move", true};

  // //actionlib::SimpleActionClient<franka_gripper::Grasp> gripper_grasp_
  // //{"/franka_gripper/grasp", true};
  
  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_grasp_{"/franka_gripper/grasp", true};
  // //actionlib::SimpleActionClient<franka_gripper::HomingAction> gripper_homing_{"/franka_gripper/homing", true};

  franka_gripper::GraspGoal goal;

  int check_gripper;

  bool check_contact;
  bool check_orientation_;
  bool check_spiral_done_;
  bool check_curved_approach_;
  bool check_yaw_motion_;

  bool is_check_orientation_first_;
  bool is_check_contact_first_;
  bool is_spiral_motion_first_;
  bool is_curved_approach_first_;
  bool is_yaw_motion_first_;
  bool is_first_;

  bool rotation_z_direction_;

  int pin_state_;


  Eigen::Matrix<double, 3, 3> rotation_z_theta_real_;
  double ori_theta_z_real_;
  double moment_xy;


};

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

        m_star = (1.0) * 350.0 * delphi_delta + 5.0 * (-current_velocity.tail<3>());

        return m_star;
    }

}  // namespace advanced_robotics_franka_controllers
