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

class TorqueJointSpaceControllerHip : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

  void approach(double current_force, double threshold, Eigen::Vector3d x, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d ori);
  void rasterSearch(Eigen::Vector3d x, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d rot);
  void updateRasterSearch();
  void insert(double current_force, double threshold, Eigen::Vector3d x, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d ori);
  Eigen::Vector3d forceSmoothing(const Eigen::Vector3d start_force, const Eigen::Vector3d target_force, const int tick, const double duration);
  void verify(const Eigen::Vector3d force_ee, const double threshold, Eigen::Vector3d x, Eigen::Matrix<double, 6, 1> xd, Eigen::Matrix3d ori);

 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  

  ros::Time start_time_;

  franka_hw::TriggerRate print_rate_trigger_{10}; 
  franka_hw::TriggerRate data_save_trigger_{1}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;
  Eigen::Vector3d pos_init_;
  Eigen::Matrix<double, 3, 3> ori_init_;
  Eigen::Matrix<double , 12, 1> x_temp_;

  //---------------my edit-----------------------------------------
  ros::Time cur_time_;
  ros::Time search_start_time_;
  ros::Time insert_start_time_;
  int status_ = 0; //0:approach, 1:search and insert, 2:done
  int assemble_dir_; //0:x, 1:y, 2:z;
  Eigen::Matrix<double, 6, 1> f_measured_;
  Eigen::Matrix<double, 6, 1> f_star_zero_;

  Eigen::Vector3d goal_position_;
  Eigen::Vector3d check_assembly_; // check assembly state along x, y,z direction w.r.t EE
  std::vector<double> fx_ee_;
  
  int cnt_ = 1;
  int sgn_ = -1;
  double range_ = 0.01;
  double r_ = 0.9;
  double duration_ = 5.0;

  bool is_first_;
  bool insert_first_;


  FILE *joint0_data;
  FILE *force_moment_ee;
  FILE *vel_ang_ee;
  FILE *force_select;
  FILE *pos_ee;

};

}  // namespace advanced_robotics_franka_controllers
