#!/usr/bin/python

import fileinput,re  
import sys

# TODO: Replace this to your package name
package_name = "advanced_robotics_franka_controllers" 

def convert(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def modify_file(file_name,pattern,value="",at_the_end=True):  
    fh=fileinput.input(file_name,inplace=True)  
    for line in fh:  
        if at_the_end:
            replacement=line+value
        else:
            replacement=value+line
        line=re.sub(pattern,replacement,line)  
        sys.stdout.write(line)  
    fh.close()  

def append_file(file_name, value):
    f = open(file_name,'a')
    f.write(value)
    f.close()

def make_file(file_name, value):
    f = open(file_name,'w')
    f.write(value)
    f.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print "How to use"
        print sys.argv[0], "ControllerName"
    elif len(sys.argv) == 2:
        controller_name = sys.argv[1]
        print "making controller:", controller_name

        modify_file("CMakeLists.txt",
                   "add_library\(\$\{PROJECT_NAME\}",
                   "src/" + convert(controller_name) + ".cpp", True)
        modify_file(package_name + "_plugin.xml",
                   "\</library\>",
                   "  <class name=\"" + package_name + "/" + controller_name + "\" type=\"" + package_name + "::" + controller_name + """\" base_class_type=\"controller_interface::ControllerBase\">
    <description>
      A controller that automatically generated by suhan script
    </description>
  </class>\n""", False)


        append_file("config/" + package_name + ".yaml", """
""" + convert(controller_name) + """:
    type: """ + package_name + '/' + controller_name + """
    arm_id: panda
    joint_names:
        - panda_joint1
        - panda_joint2
        - panda_joint3
        - panda_joint4
        - panda_joint5
        - panda_joint6
        - panda_joint7""")
        make_file("launch/{0}.launch".format(convert(controller_name)),
"""<?xml version="1.0" ?>
<launch>
  <arg name="robot_ip" default="172.16.2.2" />
  <arg name="load_gripper" default="true" />
  <include file="$(find franka_control)/launch/franka_control.launch" >
    <arg name="robot_ip" value="$(arg robot_ip)" />
    <arg name="load_gripper" value="$(arg load_gripper)" />
  </include>

  <rosparam command="load" file="$(find {0})/config/{0}.yaml" />
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen"  args="{1}"/>
  <node pkg="rviz" type="rviz" output="screen" name="rviz" args="-d $(find {0})/launch/robot.rviz"/>
</launch>
""".format(package_name,convert(controller_name)))
        make_file('include/{0}/{1}.h'.format(package_name,convert(controller_name)),
"""
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

namespace {0} {{

class {1} : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {{
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private: 
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  ros::Time start_time_;

  franka_hw::TriggerRate print_rate_trigger_{{10}}; 
									   
  Eigen::Matrix<double, 7, 1> q_init_;
  Eigen::Affine3d transform_init_;

}};

}}  // namespace {0}
""".format(package_name,controller_name))
        make_file('src/{0}.cpp'.format(convert(controller_name)),
"""
#include <{0}/{2}.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include "math_type_define.h"

namespace {0}
{{

bool {1}::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{{
	std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {{
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }}
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {{
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }}

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {{
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }}
  try {{
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  }} catch (hardware_interface::HardwareInterfaceException& ex) {{
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }}

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {{
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }}
  try {{
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  }} catch (hardware_interface::HardwareInterfaceException& ex) {{
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }}

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {{
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }}
  for (size_t i = 0; i < 7; ++i) {{
    try {{
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    }} catch (const hardware_interface::HardwareInterfaceException& ex) {{
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }}
  }}
  return true;
}}

void {1}::starting(const ros::Time& time) {{
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {{
    q_init_(i) = joint_handles_[i].getPosition();
  }}
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  
}}


void {1}::update(const ros::Time& time, const ros::Duration& period) {{
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  const std::array<double, 42> &jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  const std::array<double, 7> &gravity_array = model_handle_->getGravity();
  const std::array<double, 49> &massmatrix_array = model_handle_->getMass();
  const std::array<double, 7> &coriolis_array = model_handle_->getCoriolis();



  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(massmatrix_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> qd(robot_state.dq.data());

  Eigen::Matrix<double , 6, 7> jacobian_euler;
  Eigen::Matrix<double , 12, 7> jacobian_dc;
  Eigen::Matrix<double , 7, 1> q_goal;
  Eigen::Matrix<double , 7, 1> q_desired;
  Eigen::Matrix<double , 7, 1> qd_desired;
  Eigen::Matrix<double , 12, 1> x_goal; 
  Eigen::Matrix<double , 12, 1> x_desired;
  Eigen::Matrix<double , 12, 1> x_current;
  
  q_goal.setZero();
  q_goal << 0,0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
  q_desired.setZero();
	
  ros::Duration simulation_time = time - start_time_;
  Eigen::Matrix<double, 7, 1> tau_cmd;
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation());

  double trajectory_time = 5.0;
  for(int i=0; i<7;i++)
  {{
    q_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
                                        q_init_(i), q_goal(i), 0, 0);
    qd_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
                                        q_init_(i), q_goal(i), 0, 0);
  }}
  double kp, kv;
  kp = 1500;
  kv = 10;
  tau_cmd = mass_matrix * ( kp*(q_desired - q) + kv*(qd_desired - qd)) + coriolis;

  if (print_rate_trigger_()) {{
    ROS_INFO("--------------------------------------------------");
    ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
    //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    ROS_INFO_STREAM("time :"<< simulation_time);
    ROS_INFO_STREAM("q_curent : "<< q.transpose());
    ROS_INFO_STREAM("q_desired : "<< q_desired.transpose());


  }}

  for (size_t i = 0; i < 7; ++i) {{
    joint_handles_[i].setCommand(tau_cmd(i));
  }}

}}


}} // namespace {0}



PLUGINLIB_EXPORT_CLASS({0}::{1},
                       controller_interface::ControllerBase)
""".format(package_name,controller_name,convert(controller_name)))