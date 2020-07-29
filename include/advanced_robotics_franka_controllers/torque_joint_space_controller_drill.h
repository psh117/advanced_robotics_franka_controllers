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

// #define READY -1
// #define APPROACH 0
// #define SEARCH 1
// #define INSERT 2
// #define ESCAPE 3
// #define BACK 4
// #define RELEASE 5
enum STATE {
  READY, 
  APPROACH, 
  SEARCH, 
  INSERT, 
  ESCAPE, 
  BACK, 
  RELEASE
};


namespace advanced_robotics_franka_controllers {

class TorqueJointSpaceControllerDrill : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void approach(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d ori);
  // void search(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd);
  // void insert(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd);
  // void release(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rotation);
  // void escape(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd);
  // void back(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd);
  // void moveToRandomPoint(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd);
  void gripperClose();
  void gripperOpen();

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
  Eigen::Matrix<double , 12, 1> x_temp_;


  FILE *force_moment_ee;
  FILE *cmd_task_space;
  FILE *cmd_joint_space;



  Eigen::Vector3d target_x_;
  Eigen::Vector3d x_desired_;
  Eigen::Vector3d xdot_desired_;
  
	Eigen::Matrix<double, 6, 1> f_star_zero_;

  Eigen::Matrix<double, 6, 1> target_f;

  Eigen::Matrix<double, 3, 7> jacobian_pos_;

  Eigen::Matrix<double, 6, 6> lambda_;
  Eigen::Matrix<double, 3, 3> lambda_v_;

  Eigen::Vector3d delphi_delta;

  ros::Duration finish_time;

  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_ac_close_
  {"/franka_gripper/grasp", true};
  
  actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_ac_open_
  {"/franka_gripper/move", true};

  // //actionlib::SimpleActionClient<franka_gripper::Grasp> gripper_grasp_
  // //{"/franka_gripper/grasp", true};
  
  // actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_grasp_{"/franka_gripper/grasp", true};
  // //actionlib::SimpleActionClient<franka_gripper::HomingAction> gripper_homing_{"/franka_gripper/homing", true};

  franka_gripper::GraspGoal goal;

 
  // ------

  int status_;
  int assembly_dir_;
  int assembly_dir_ee_;

  bool is_ready_;

  bool is_random_first_;
  bool is_approach_first_;
  bool is_search_first_;
  bool is_insert_first_;
  bool is_release_first_;
  bool is_escape_first_;
  bool is_back_first_;
  
  bool is_random_done_;
  bool is_approach_done_;
  bool is_search_done_;
  bool is_insert_done_;
  bool is_escape_done_;
  bool is_release_done_;
  bool is_back_done_;


  ros::Time cur_time_;
  ros::Time approach_start_time_;
  ros::Time spiral_start_time_;
  ros::Time insert_start_time_;
  ros::Time release_start_time_;
  ros::Time escape_start_time_;
  ros::Time back_start_time_;
  
  Eigen::Vector3d init_force_;
  Eigen::Vector3d goal_force_;
  Eigen::Vector3d init_moment_;
  Eigen::Vector3d goal_moment_;

  Eigen::Vector3d origin_;

  Eigen::Vector3d pos_random_;
  Eigen::Vector3d goal_;

  Eigen::Matrix<double, 6, 1> f_measured_;
};

}  // namespace advanced_robotics_franka_controllers