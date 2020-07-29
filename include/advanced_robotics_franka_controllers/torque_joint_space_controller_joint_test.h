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

// enum STATE = {
//   READY = -1, 
//   TILT = 0, 
//   MOVEBACK = 1, 
//   APPROACH = 2, 
//   SEARCH = 3, 
//   INSERT = 4, 
//   RELEASE = 5
// };
enum STATE {
  READY, 
  TILT, 
  MOVEBACK, 
  APPROACH, 
  SEARCH, 
  INSERT, 
  RELEASE
};

namespace advanced_robotics_franka_controllers {

class TorqueJointSpaceControllerJointTest : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void ready(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd,
              const Eigen::Matrix<double, 6, 1> f_measured, const Eigen::Matrix3d base_rotation);
  void tilt(const Eigen::Vector3d position, const Eigen::Matrix<double, 6, 1> xd, const Eigen::Matrix3d rotation,
              const double tilt_angle, const int tilt_axis, const double duration);
  void moveback(const Eigen::Vector3d position, const Eigen::Matrix<double, 6, 1> xd, const Eigen::Matrix3d rotation,
                const double duration);
  void approach(Eigen::Vector3d position, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rotation);
  void search(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd);
  void insert(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd);
  void release(const Eigen::Vector3d position, const Eigen::Matrix3d rotation, const Eigen::Matrix<double, 6, 1> xd);
  void gripperClose();

 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  

  ros::Time start_time_;
  ros::Time cur_time_;
  ros::Time tilt_start_time_;
  ros::Time moveback_start_time_;
  ros::Time approach_start_time_;
  ros::Time spiral_start_time_;
  ros::Time insert_start_time_;
  ros::Time release_start_time_;

  franka_hw::TriggerRate print_rate_trigger_{10}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;
  Eigen::Vector3d pos_init_;
  Eigen::Vector3d the_origin_;
  Eigen::Matrix<double, 3, 3> ori_init_;
  Eigen::Matrix<double , 12, 1> x_temp_;
  Eigen::Matrix3d base_rotation_; // the orientation of the base component!!
  Eigen::Matrix3d search_pose_rotation_;
  Eigen::Vector3d f_ee_lpf_;
  Eigen::Vector3d m_ee_lpf_;
  Eigen::Vector3d f_ee_prev_;
  Eigen::Vector3d m_ee_prev_;

  Eigen::Vector3d f_asm_;

  Eigen::Vector3d assembly_dir_vec_;
  Eigen::Vector3d tilt_axis_;

  FILE *joint0_data;
  FILE *pr_real;
  FILE *fm_real;
  FILE *fm_cmd;
  FILE *torque_cmd;
  FILE *spiral_position;
  
  Eigen::Matrix<double, 6, 1> f_star_zero_;


  bool is_ready_first_;
  bool is_tilt_first_;
  bool is_moveback_first_;
  bool is_approach_first_; //1
  bool is_search_first_;    //2
  bool is_insert_first_;    //3
  bool is_release_first_;

  bool set_tilt_;

  bool pause_;

  STATE state_;
  int assembly_dir_;

  double angle_;

  double ready_duration_;
  double tilt_duration_;
  double moveback_duration_;

  double holding_force_;

  int contact_check_cnt_;

  // model information
  Eigen::Vector3d assembly_pos_;
  Eigen::Vector4d assembly_quat_;
  Eigen::Matrix4d assembly_frame_;

  Eigen::Vector3d grasp_pos_;
  Eigen::Vector4d grasp_quat_;
  Eigen::Matrix3d grasp_rot_;
  Eigen::Matrix4d grasp_frame_;

  Eigen::Matrix4d T_GA_;
  
  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_ac_close_
  {"/franka_gripper/grasp", true};
};

}  // namespace advanced_robotics_franka_controllers
