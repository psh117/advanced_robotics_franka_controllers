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
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
/////////////////////////////////// 
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
/////////////////////////////////////////

namespace advanced_robotics_franka_controllers {
using namespace Eigen;

class TorqueJointSpaceControllerRRT : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
 void traj_cb(const sensor_msgs::JointStateConstPtr& msg);
  void planned_cb(const std_msgs::Bool& msg);
  void grip_cb(const std_msgs::Bool& msg);
  void grip_open_cb(const std_msgs::Bool& msg);

/////////////////////////////////////////
  void commandForceCallback(const geometry_msgs::WrenchConstPtr& msg);
  void pegInHoleStateCallback(const std_msgs::Int32ConstPtr& msg);
  void taskStartCallback(const std_msgs::Bool& msg);
///////////////////////////////////////////


  Matrix3d Rotate_with_X(const double rAngle);
Matrix3d Rotate_with_Y(const double rAngle);
Matrix3d Rotate_with_Z(const double rAngle);


 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  bool planned_done;

  bool gripper_done;
  bool gripper_open;
  ros::Time start_time_;

  // ros::Publisher joint_state_pub_;
  ros::Subscriber planned_trajectory;
  ros::Subscriber planned_done_;
  ros::Subscriber gripper_sub_;
  ros::Subscriber gripper_open_sub_;

  // ros::Publisher goal_state_pub_;

  ////////////////////////// peg in hole ///////////////////////////////

 ros::Publisher current_velocity_pub_; //linear + angular

  ros::Publisher current_position_pub_;
  ros::Publisher current_orientation_pub_;
  ros::Publisher current_force_pub_;
  ros::Publisher current_torque_pub_;
	  
  ros::Subscriber f_star_zero_sub_;
  ros::Subscriber peg_in_hole_state_sub_;
  ros::Subscriber task_start_sub_;
    
  std_msgs::Float32MultiArray current_velocity_info_;
  geometry_msgs::Point current_position_info_;
  std_msgs::Float32MultiArray current_orientation_info_;
  geometry_msgs::Wrench current_force_info_;
  geometry_msgs::Wrench current_torque_info_;
    

  Eigen::Matrix<double, 6, 1> f_star_zero_;
  int peg_in_hole_state_;

  Eigen::Matrix<double, 6, 1> current_velocity_; //linear + angular
  Eigen::Vector3d current_position_;
  Eigen::Vector3d current_orientation_;
  Eigen::Vector3d current_force_;
  Eigen::Vector3d current_torque_;

  Eigen::Matrix<double, 6, 1> f_sensing;

  bool task_start_;
  bool the_first_tick_;
  bool is_first_;

  Eigen::Vector3d desired_position_;
  Eigen::Matrix<double, 3, 3> desired_rotation_M;
  Eigen::Matrix<double, 3, 3> K_p;
  Eigen::Matrix<double, 3, 3> K_v;
  Eigen::Vector3d delphi_delta_;

  ///////////////////////////////////////////////////////////////////





  realtime_tools::RealtimePublisher<sensor_msgs::JointState> joint_state_pub_;


  sensor_msgs::JointState current_joint_state_;
  geometry_msgs::Transform goal_trans_state_;

  franka_hw::TriggerRate print_rate_trigger_{10}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;
  Eigen::Vector3d pos_init_;
  Eigen::Matrix<double, 3, 3> ori_init_;
  Eigen::Matrix<double , 12, 1> x_temp_;

    Eigen::Matrix<double, 7, 1> q_traj_;



  FILE *joint0_data;

  actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_ac_
  {"/franka_gripper/grasp", true};
  actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_ac_open
  {"/franka_gripper/move", true};
};

}  // namespace advanced_robotics_franka_controllers
