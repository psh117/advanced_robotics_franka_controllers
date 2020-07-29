
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

namespace advanced_robotics_franka_controllers {

class SuhanController : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {

  enum class ControlType
  {
    None, PathFollowing, Assembly1, Assembly2
  };

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

  void initTasks();
  void getTrajectories();

  Eigen::Matrix<double, 7, 1> movePathUpdate(const ros::Time& time, Eigen::Matrix<double, 7, 1> target_pos);
  Eigen::Matrix<double, 7, 1> assembleUpdate(const ros::Time& time, SuhanController::ControlType assembly_type);

  std::vector<std::pair<double , ControlType>> tasks_; // time, ctr type
  int task_index_ {0};
  int traj_index_ {0};

  ControlType time2task(const ros::Time& time);
 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;



  ros::Time start_time_;
  ros::Time task_start_time_;

  franka_hw::TriggerRate print_rate_trigger_{10}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;

  
  double check_stop_assemlby_;
  Eigen::Matrix<double, 3, 3> ori_init_assembly_;
  double time_starting_assembly_;
  Eigen::Vector3d spiral_starting_pos_assembly_;

};

}  // namespace advanced_robotics_franka_controllers
