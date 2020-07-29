#include <advanced_robotics_franka_controllers/torque_joint_space_controller_revolve.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka/model.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>



namespace advanced_robotics_franka_controllers
{

bool TorqueJointSpaceControllerRevolve::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  //joint0_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint0_data.txt","w");
  save_data_x = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_data_fm.txt","w");   
  save_data_x2 = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_data_pv.txt","w");
  save_result = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_result.txt","w");
  save_dir = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_dir.txt","w");
  save_cmd = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_cmd.txt","w");
  std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

void TorqueJointSpaceControllerRevolve::starting(const ros::Time& time) {
  start_time_ = time;
	
  for (size_t i = 0; i < 7; ++i) {
    q_init_(i) = joint_handles_[i].getPosition();
  }
  
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  pos_init_ = transform_init_.translation();	
  ori_init_ = transform_init_.rotation();
  xdot_desired_.setZero();

  f_star_.setZero();
  m_star_.setZero();

  is_first_ = true;
  rotate_is_done_ = false;
  f_sum_ = 0.0;
  cnt_ = 0;
  dir_ = 0;
}


void TorqueJointSpaceControllerRevolve::update(const ros::Time& time, const ros::Duration& period) {

  const franka::RobotState &robot_state = state_handle_->getRobotState();
  const std::array<double, 42> &jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  const std::array<double, 7> &gravity_array = model_handle_->getGravity();
  const std::array<double, 49> &massmatrix_array = model_handle_->getMass();
  const std::array<double, 7> &coriolis_array = model_handle_->getCoriolis();

  const std::array<double, 3ul> gravity_dir = {{0., .0, 9.81}};
  double theta = 15.0;

  std::array<double, 7> gravity2 = model_handle_->getGravity({{9.81*sin(theta*M_PI/180.0), 0.0, -9.81*cos(theta*M_PI/180.0)}});//franka::Model::gravity(robot_state, gravity_dir);

  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(massmatrix_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> qd(robot_state.dq.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> graivity_mod(gravity2.data());

  Eigen::Matrix<double , 6, 7> jacobian_euler;
  Eigen::Matrix<double , 12, 7> jacobian_dc;
  Eigen::Matrix<double , 7, 1> q_goal;
  Eigen::Matrix<double , 7, 1> q_desired;
  Eigen::Matrix<double , 7, 1> qd_desired;
  Eigen::Matrix<double , 12, 1> x_goal; 
  Eigen::Matrix<double , 12, 1> x_desired;
  Eigen::Matrix<double , 12, 1> x_current;

  Eigen::Matrix<double, 6, 6> lambda;
  Eigen::Matrix<double, 7, 6> j_bar;

  lambda = (jacobian*mass_matrix.inverse()*jacobian.transpose()).inverse();
  j_bar = mass_matrix.inverse()*jacobian.transpose()*lambda;

  q_goal.setZero();
  q_goal << 0.0, -M_PI/6, 0.0, -2*M_PI/3, 0, M_PI/2, M_PI/4;
  q_desired.setZero();

  ros::Duration simulation_time = time - start_time_;
  Eigen::Matrix<double, 7, 1> tau_cmd;
	
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation()); 

  Eigen::Vector6d x_dot_(jacobian*qd);

  double trajectory_time = 5.0;
  double duration = 10.0;
  double dis = -0.02;
  
  for(int i=0; i<7;i++)
  {
    q_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
                                        q_init_(i), q_goal(i), 0, 0);
    qd_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec(), start_time_.toSec() + trajectory_time,
                                        q_init_(i), q_goal(i), 0, 0);
  }
  
  f_sensing_ = j_bar.transpose()*(tau_measured - gravity);
  f_sensing_ee_.head<3>() = rotation_M.transpose()*f_sensing_.head<3>();
  f_sensing_ee_.tail<3>() = rotation_M.transpose()*f_sensing_.tail<3>();
  ////////////////////
  
  if(time.toSec() - start_time_.toSec() < 0.5)
  {
    f_star_ = keepCurrentPosition(pos_init_, position, x_dot_);
    m_star_ = keepCurrentOrientation(ori_init_, rotation_M, x_dot_);     
    f_sum_ += f_sensing_ee_(2);
    cnt_++;
  }

  else
  {
    // to rotate to align angle ----------
    
    if(rotate_is_done_ == false)
    {
      f_star_ = keepCurrentPosition(pos_init_, position, x_dot_); // align z-axis angle!
      m_star_ = rotateWithLocalAxis(ori_init_, rotation_M, x_dot_, -2.0*M_PI/180, time.toSec(), start_time_.toSec(), 2); 
      if(time.toSec() - start_time_.toSec() >= 5.5) rotate_is_done_ = true; 
    }    
    // ---------- to rotate to align angle

    // to detect a pin on back_side -----------
    else
    {
      if(is_first_ == true)
      {  
        pos_init_ = position;
        ori_init_ = rotation_M;
        start_time_ = time;
        is_first_ = false;
        std::cout<<"dir is "<<dir_<<std::endl;
      }

      double run_time = time.toSec() - start_time_.toSec();

      switch(dir_)
      {
        case 0:
          f_star_ = oneDofMoveEE(pos_init_, ori_init_, position, x_dot_, time.toSec(), start_time_.toSec(), duration, dis, 1); // searching pins 
          
          if(run_time >= duration)
          {
            is_first_ = true;
            dir_ = 1;
          }
          break;

        case 1:
          f_star_ = oneDofMoveEE(pos_init_, ori_init_, position, x_dot_, time.toSec(), start_time_.toSec(), duration*2, -dis*2, 1); // searching pins 

          if(run_time >= duration*2)
          {
            is_first_ = true;
            dir_ = 2;
          }
          break;

        case 2:   
          f_star_ = oneDofMoveEE(pos_init_, ori_init_, position, x_dot_, time.toSec(), start_time_.toSec(), duration*2, dis*2, 1); // searching pins 
          
          if(run_time >= duration*2)
          {
            is_first_ = true;
            dir_ = 1;
          }   
          break;
      } 
  
      f_star_(2) = -15.0;  

      m_star_ = keepCurrentOrientation(ori_init_, rotation_M, x_dot_);     
  
      if(fabs(f_sensing_ee_(2) - f_sum_/cnt_) > 2.0)
      {
        // f_star_.setZero();
        // m_star_.setZero();
        std::cout<<f_sensing_ee_(2)<<" "<<f_sum_/cnt_<<std::endl;
        std::cout<<"DETECT HOLES"<<std::endl; 
      }
      // ----------- to detect a pin on back_side 
      }    
  }
  
  

  f_star_zero_.head<3>() = f_star_;
  f_star_zero_.tail<3>() = m_star_;
  
  tau_cmd = jacobian.transpose() * (f_star_zero_);

  if (print_rate_trigger_()) {
    // ROS_INFO("--------------------------------------------------");
    // ROS_INFO_STREAM("f_star_zero_ : "<< f_star_zero_.transpose());
    // ROS_INFO_STREAM("x_desired : "<< x_desired_.transpose());
    // ROS_INFO_STREAM("mass :" << mass_matrix)s
    // ROS_INFO_STREAM("q :"<< q.transpose());
    // ROS_INFO_STREAM("f_sensing : "<< f_sensing.transpose());
    // ROS_INFO_STREAM("f_star_ : "<< f_star_.transpose());
    // ROS_INFO_STREAM("m_star_ : "<< m_star_.transpose());
    // ROS_INFO_STREAM("rotation_M : "<< rotation_M);

  }

  fprintf(save_data_x2, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", position(0), position(1), position(2), x_dot_(0), x_dot_(1), x_dot_(2), x_dot_(3), x_dot_(4), x_dot_(5));
  fprintf(save_cmd, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_sensing_ee_(0), f_sensing_ee_(1), f_sensing_ee_(2), f_sensing_ee_(3), f_sensing_ee_(4), f_sensing_ee_(5));
  
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

}


} // namespace advanced_robotics_franka_controllers

PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerRevolve,
                       controller_interface::ControllerBase)
