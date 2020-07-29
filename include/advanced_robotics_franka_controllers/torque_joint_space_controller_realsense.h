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
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

namespace advanced_robotics_franka_controllers {

class TorqueJointSpaceControllerRealsense : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

  void targePointCallback(const geometry_msgs::PoseArrayPtr &msg);


 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  ros::Subscriber target_3d_points_sub_;
  ros::Time start_time_;

  franka_hw::TriggerRate print_rate_trigger_{10}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;
  Eigen::Vector3d pos_init_;
  Eigen::Matrix<double, 3, 3> ori_init_;
  Eigen::Matrix<double , 12, 1> x_temp_;

  Eigen::Vector3d cam_p_obj_;
  Eigen::Vector3d ee_p_cam_;
  Eigen::Vector3d uni_p_obj_;
  Eigen::Vector3d uni_p_ee_;

  Eigen::Matrix3d cam_r_obj_;
  Eigen::Matrix3d ee_r_cam_;
  Eigen::Matrix3d uni_r_obj_;
  Eigen::Matrix3d uni_r_ee_;

  Eigen::Matrix4d cam_T_obj_;
  Eigen::Matrix4d ee_T_cam_;
  Eigen::Matrix4d uni_T_obj_;
  Eigen::Matrix4d uni_T_ee_;

  std::vector<double> obj_x_;
  std::vector<double> obj_y_;
  std::vector<double> obj_z_;

  FILE *joint0_data;

};

}  // namespace advanced_robotics_franka_controllers
