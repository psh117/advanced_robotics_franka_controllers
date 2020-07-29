
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

#include "math_type_define.h"

namespace advanced_robotics_franka_controllers {

class VelocityJointSpaceController : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   //hardware_interface::EffortJointInterface,
                   hardware_interface::VelocityJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  ros::Time start_time_;

  franka_hw::TriggerRate print_rate_trigger_{10}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;

  FILE *joint0_data;
  FILE *joint_cmd;
  FILE *joint_real;


  std::ifstream q_path;
  double q_num = 25000;
  // Eigen::VectorXd q_data[2];
  std::vector<Eigen::Vector7d> teaching_joint_datas;

  int j = 0;
  Eigen::Vector7d last_data;
  double data_size = 0;

  Eigen::Matrix<double, 7, 1> q_goal_;
  Eigen::Matrix<double, 7, 1> q_goal_next;

  std::string file_path = "/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE/";

};

}  // namespace advanced_robotics_franka_controllers
