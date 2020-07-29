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

namespace advanced_robotics_franka_controllers {

class TorqueJointSpaceControllerDualSpiral : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void approach(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d ori);
  void search(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd);
  void insert(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd);
  void release(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rotation);
  void gripperOpen();

 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;


  //std::unique_ptr<franka_gripper::grasp> gripper_;
  

  ros::Time start_time_;
  ros::Time time_ori_0;

  franka_hw::TriggerRate print_rate_trigger_{10}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;
  Eigen::Vector3d pos_init_;
  Eigen::Matrix<double, 3, 3> ori_init_;
  Eigen::Matrix<double , 12, 1> x_temp_;


  FILE *joint0_data;
  FILE *save_data_x;
  FILE *save_data_x2;
  FILE *force_moment_ee;
  FILE *cmd_task_space;
  FILE *cmd_joint_space;

  FILE *save_force;
  FILE *save_position;
  FILE *save_velocity;

  Eigen::Vector3d target_x_;
  Eigen::Vector3d x_desired_;
  Eigen::Vector3d xdot_desired_;
  
  Eigen::Matrix<double, 3, 3> K_p;
	Eigen::Matrix<double, 3, 3> K_v;
  Eigen::Matrix<double, 3, 3> K_p_ori;

  Eigen::Vector3d f_star_;
	Eigen::Vector3d m_star_;
	Eigen::Matrix<double, 6, 1> f_star_zero_;

  Eigen::Matrix<double, 6, 1> target_f;

  Eigen::Matrix<double, 3, 7> jacobian_pos_;

  Eigen::Matrix<double, 6, 6> lambda_;
  Eigen::Matrix<double, 3, 3> lambda_v_;

  Eigen::Matrix<double, 3, 3> target_rotation_, rotation_z_theta_, rotation_z_theta_2_, rotation_x_theta_;
  double ori_theta, ori_theta_init;
  int ori_change_direction;
  int ori_check_time;
  Eigen::Matrix<double, 3, 3> rotation_0, rotation_f;
  double ori_duration;
  int f_sensing_read_check;


  Eigen::Vector3d delphi_delta;

  int check_stop;
  Eigen::Vector3d stop_x;
  int check_fake;

  Eigen::Matrix<double, 6, 1> f_sensing_read;

  ros::Duration finish_time;

  actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_ac_
  {"/franka_gripper/move", true};

  // //actionlib::SimpleActionClient<franka_gripper::Grasp> gripper_grasp_
  // //{"/franka_gripper/grasp", true};
  
  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_grasp_{"/franka_gripper/grasp", true};
  // //actionlib::SimpleActionClient<franka_gripper::HomingAction> gripper_homing_{"/franka_gripper/homing", true};

  franka_gripper::GraspGoal goal;

  int check_gripper;

  bool check_contact;

  //--renew the code from here---
  int status_;
  int assembly_dir_;
  int assembly_dir_ee_;
  
  bool is_approach_first_;
  bool is_search_first_;
  bool is_insert_first_;
  bool is_release_first_;
  bool is_first_;
  bool is_ready_;
  
  bool is_approach_done_;
  bool is_search_done_;
  bool is_insert_done_;
  bool is_release_done_;

  ros::Time cur_time_;
  ros::Time spiral_start_time_;
  ros::Time insert_start_time_;
  ros::Time release_start_time_;
  ros::Time ori_start_time_;

  int ori_change_dir_;

  int smoothing_tick_ = 0;
  int hz_ = 1000;

  Eigen::Vector3d init_force_;
  Eigen::Vector3d goal_force_;
  Eigen::Vector3d init_moment_;
  Eigen::Vector3d goal_moment_;

  Eigen::Vector3d assembly_pos_;
  Eigen::Vector4d assembly_quat_;
  Eigen::Matrix4d assembly_frame_;

  Eigen::Vector3d grasp_pos_;
  Eigen::Vector4d grasp_quat_;
  Eigen::Matrix3d grasp_rot_;
  Eigen::Matrix4d grasp_frame_;

  Eigen::Matrix4d T_GA_;
  Eigen::Matrix4d T_EA_;

  Eigen::Vector3d assembly_dir_vec_;
};

}  // namespace advanced_robotics_franka_controllers
