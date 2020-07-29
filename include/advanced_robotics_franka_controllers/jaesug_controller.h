
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
#include <advanced_robotics_franka_controllers/robot_model.h>

namespace advanced_robotics_franka_controllers {

class JaesugController : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  Eigen::Matrix<double, 6, 1> getfstar();
  Eigen::Matrix<double, 6, 1> getfstar2();
  Eigen::Matrix<double, 6, 1> getfstar3();
  Eigen::Matrix<double, 7, 1> getgstar(Eigen::Matrix<double, 7, 1> q, Eigen::Matrix<double, 7, 1> qd);
  Eigen::Matrix<double, 7, 1> calctasktorque(Eigen::Matrix<double, 6, 7> Jtask, Eigen::Matrix<double, 6, 1> fstar);
  Eigen::Matrix<double, 7, 1> calcjointtorque(Eigen::Matrix<double, 7, 1> gstar);
  void clik(Eigen::Matrix<double, 6, 7> Jtask);
  void dyn_consist_ik(Eigen::Matrix<double, 6, 7> Jtask);

 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  ros::Time start_time_;

  franka_hw::TriggerRate print_rate_trigger_{10}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;
  Eigen::Matrix<double, 7, 1> dq_filtered_;
  Eigen::Matrix<double, 6, 1> control_state;
  Eigen::Matrix<double, 6, 1> control_state_dot;
  Eigen::Matrix<double, 6, 1> desired_state;
  Eigen::Matrix<double, 6, 1> desired_state_dot;
  Eigen::Matrix<double, 6, 1> fstar;
  Eigen::Matrix<double , 7, 1> q_goal;
  Eigen::Matrix<double , 7, 1> q_desired;
  Eigen::Matrix<double , 7, 1> qd_desired;
  Eigen::Matrix<double , 7, 1> gstar;
  RobotModel * robot_;
  RobotModel * robot_test;
  Eigen::Vector3d pos_ee;
  Eigen::Vector3d pos_virtual1;
  Eigen::Vector3d pos_virtual2;
  Eigen::Vector3d pos_wrist;
  Eigen::VectorXd pos_ee_dot;
  Eigen::VectorXd pos_virtual1_dot;
  Eigen::VectorXd pos_virtual2_dot;
  Eigen::VectorXd pos_wrist_dot;
  Eigen::Vector3d pos_ee_init;
  Eigen::Vector3d pos_virtual1_init;
  Eigen::Vector3d pos_virtual2_init;
  Eigen::Vector3d pos_wrist_init;
  Eigen::Vector3d pos_ee_goal;
  Eigen::Vector3d pos_wrist_goal;
  Eigen::Vector3d pos_ee_desired;
  Eigen::Vector3d pos_ee_desired_pre;
  Eigen::Vector3d pos_ee_dot_desired;
  Eigen::Vector3d pos_wrist_desired;
  Eigen::Vector3d pos_wrist_dot_desired;
  Eigen::Vector3d pos_ee_desired2;
  Eigen::Vector3d pos_ee_dot_desired2;
  Eigen::Vector3d pos_ee_desired3;
  Eigen::Vector3d pos_ee_dot_desired3;
  Eigen::Vector3d angle_init;
  Eigen::Vector3d angle_goal;
  Eigen::Vector3d angle_desired;
  Eigen::Vector3d angle_wrist_init;
  Eigen::Vector3d angle_wrist_goal;
  Eigen::Vector3d angle_wrist_desired;
  Eigen::Vector3d angle_dot_desired;
  Eigen::Vector3d angle_wrist_dot_desired;
  Eigen::Vector3d angle;
  Eigen::Vector3d angle_wrist;
  Eigen::Vector3d angle_test;
  Eigen::Vector3d tip1;
  Eigen::Vector3d tip2;
  Eigen::Vector3d tip3;
  Eigen::Vector3d tipVector1;
  Eigen::Vector3d tipVector2;
  Eigen::MatrixXd J_task;
  Eigen::MatrixXd Jacob_ee;
  Eigen::MatrixXd Jacob_wrist;
  Eigen::MatrixXd Jacob_ee2;
  Eigen::MatrixXd Jacob_ee3;
  Eigen::Matrix3d Rot_cur;
  Eigen::Matrix3d Rot_wrist;
  Eigen::Matrix3d Rot_init;
  Eigen::Matrix3d Rot_wrist_init;
  Eigen::Matrix3d Rot_desired;
  Eigen::Matrix3d Rot_wrist_desired;
  Eigen::Matrix3d Rot_desired_pre;
  Eigen::Matrix3d Rot_goal;
  Eigen::Matrix3d Rot_i2g;
  double axis_angle_goal;
  double axis_angle_desired;
  Eigen::Vector3d init_angle_vector;
  Eigen::Vector3d init_angle_vector2;
  Eigen::Vector3d axis_angle_vector_goal;
  Eigen::MatrixXd mass_inv;
  Eigen::MatrixXd motor_inertia;
  Eigen::MatrixXd motor_inertia_inv;
  Eigen::MatrixXd LAMBDA;
  FILE *save_data;
  FILE *save_data2;
  FILE *save_data3;
  FILE *save_data4;

  // hosang
  Eigen::MatrixXd S_mod;
  Eigen::VectorXd fstar_mod;
  Eigen::VectorXd tau_mod;
  Eigen::MatrixXd J_mod;
  Eigen::MatrixXd Z;
  
};

}  // namespace advanced_robotics_franka_controllers
