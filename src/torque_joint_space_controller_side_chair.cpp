#include <advanced_robotics_franka_controllers/torque_joint_space_controller_side_chair.h>
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

bool TorqueJointSpaceControllerSideChair::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
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

void TorqueJointSpaceControllerSideChair::starting(const ros::Time& time) {
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

  approach_origin_.setZero();
  keep_state_origin_.setZero();
  revolve_origin_.setZero();

  initial_rotation_M.setZero();
  last_rotation_M.setZero();

  rotation_dir_.setZero();
  
  approach_start_time_ = 0;
  revolve_start_time_ = 0;
  keep_state_start_time_ = 0;

  detect_contact_ = false;

  is_approach_first_ = true;
  is_revolve_first_ = true;
  is_keep_state_first_ = true; 

  index_ = 0;
  contact_count_ = 0;
  contact_points_ = 0;
  initial_force_ = 0.0;
  final_force_= 0.0;
  initial_moment_.setZero();
  revolve_direction_ = 0;
  
}


void TorqueJointSpaceControllerSideChair::update(const ros::Time& time, const ros::Duration& period) {

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
  if(simulation_time.toSec() <= 2.0) //At the firt, freeze to stablize its posture
  {
    keepState(time, position, rotation_M, x_dot_);
  }

  else
  {
    if(contact_points_ == 0)
    {  
      if(index_<=SIZE+1)
      {
        keepState(time, position, rotation_M, x_dot_);
        m_star_.setZero();
        getInitialFT(index_);
        index_++;              
      }
      else
      {
        if(detect_contact_ == false)
        { 
          if(f_sensing_(2) <= -3.0)//-initial_force_*0.5)
          {
            contact_count_ ++;
            getDirectionVector(position, rotation_M);
          } 

          else
          {
            contact_count_ = 0;        
            clearDirectionVector();
          } 

          approach(time, position, rotation_M, x_dot_); 
        }

        else
        {
          keepState(time, position, rotation_M, x_dot_);
          m_star_.setZero();
        }
            
      }    
    }

    else if(contact_points_ == 1)
    {
      if(index_<=SIZE+1)
      {
        keepState(time, position, rotation_M, x_dot_);
        m_star_.setZero();
        getInitialFT(index_);
        index_++;    
        detect_contact_ = false;    
      }
      else
      {
        if(detect_contact_ == false)
        {
          if(f_sensing_(2) <= final_force_*1.0)
          {
            contact_count_ ++;
            getDirectionVector(position, rotation_M);
          } 
          else
          {
            contact_count_ = 0;        
            clearDirectionVector();
          } 
          
          approach(time, position, rotation_M, x_dot_); 
          m_star_.setZero();
        }
      
        else
        {
          keepState(time, position, rotation_M, x_dot_);
          m_star_.setZero();
        }

      }
    }
    else if(contact_points_ == 2)
    {
      if(index_<=SIZE+1)
      {
        if(index_ >= SIZE+1) rotation_dir_ = computeRotationAxis(first_rotation_M, last_rotation_M);
        keepState(time, position, rotation_M, x_dot_);
        m_star_.setZero();
        index_++;    
        detect_contact_ = false;    
      }

     else
      {
        if(detect_contact_ == false)
        { 
          revolve(time, rotation_dir_, position, rotation_M, x_dot_, 10*DEG2RAD, 10.0); 
          
          if(time.toSec() - revolve_start_time_ > 1.0)
          {
            double temp_mx = fabs(mx_ee_.back() - initial_moment_(0));
            double temp_my = fabs(my_ee_.back() - initial_moment_(1));
        
            if(temp_mx >= 2.5 && temp_my >= 2.5)
            {
              contact_points_++;
              std::cout<<"CHECK CONTACT, moment limit"<<std::endl;
              std::cout<<temp_mx<<" "<<temp_my<<std::endl;
              std::cout<<mx_ee_.back()<<" "<<my_ee_.back()<<std::endl;
              std::cout<<initial_moment_(0)<<" "<<initial_moment_(1)<<std::endl;
              std::cout<<time.toSec() - revolve_start_time_<<std::endl;
            }          
          }          
        } 
      
        else
        {
          keepState(time, position, rotation_M, x_dot_);
          m_star_.setZero();
        }
      }
    }
    else
    {
      keepState(time, position, rotation_M, x_dot_);
      m_star_.setZero();
    }
  }
  

  f_star_zero_.head<3>() = f_star_;
  f_star_zero_.tail<3>() = m_star_;
  
  tau_cmd = jacobian.transpose() * (f_star_zero_);
  //tau_cmd.setZero();

  if (print_rate_trigger_()) {
      // ROS_INFO("--------------------------------------------------");
      // ROS_INFO_STREAM("error :");
      // ROS_INFO_STREAM(euler_diff_);
    // // // ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
    // // // ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
    // // // ROS_INFO_STREAM("time :"<< simulation_time);
    // ROS_INFO_STREAM("x_curent : "<< position.transpose());
    // // // ROS_INFO_STREAM("x_desired : "<< x_desired_.transpose());
    // // // ROS_INFO_STREAM("mass :" << mass_matrix)s
    // // // ROS_INFO_STREAM("q :"<< q.transpose());
    // // // ROS_INFO_STREAM("f_sensing : "<< f_sensing.transpose());
    // ROS_INFO_STREAM("f_star_ : "<< f_star_.transpose());
    // ROS_INFO_STREAM("m_star_ : "<< m_star_.transpose());
    // ROS_INFO_STREAM("rotation_M : "<< rotation_M);

  }

  fprintf(save_data_x2, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", position(0), position(1), position(2), x_dot_(0), x_dot_(1), x_dot_(2), x_dot_(3), x_dot_(4), x_dot_(5));
  fprintf(save_cmd, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_star_zero_(0), f_star_zero_(1), f_star_zero_(2), f_star_zero_(3), f_star_zero_(4), f_star_zero_(5));
  
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

}

void TorqueJointSpaceControllerSideChair::approach(const ros::Time& time, 
  const Eigen::Vector3d position, 
  const Eigen::Matrix3d rotation_M, 
  const Eigen::Matrix<double, 6, 1> x_dot_)
{
  if(is_approach_first_ == true)
  {
    approach_origin_ = position;
    initial_rotation_M = rotation_M;
    approach_start_time_ = time.toSec();
    is_approach_first_ = false;
    is_keep_state_first_ = true;

    if(contact_points_ == 1)
    {
      first_rotation_M = initial_rotation_M;
      std::cout<<initial_rotation_M<<std::endl;
    }
    
  }
  
  if(contact_count_ >= 1000) // finish the function
  {
    detect_contact_ = true;
    is_approach_first_ = true;

    index_ = 0;
    contact_count_ = 0;
    contact_points_ ++;
    final_force_ = f_sensing_(2);
    std::cout<<"final force: "<<final_force_<<std::endl;
    if(contact_points_ == 2)
    {
      last_rotation_M = rotation_M;
      std::cout<<last_rotation_M<<std::endl;
    } 
    std::cout<<"CHECK CONTACT"<<std::endl;
    // computeContactPoint(initial_rotation_M, nx_, ny_, nz_, ex_, ey_, ez_, p_);
  }

  else
  {
    f_star_ = straightApproach(approach_origin_, initial_rotation_M, rotation_M, position, x_dot_, time.toSec(), approach_start_time_ ).head<3>();
    //m_star_ = straightApproach(approach_origin_, initial_rotation_M, rotation_M, position, x_dot_, time.toSec(), approach_start_time_ ).tail<3>();   
    m_star_ = keepOrientationPerpenticular(initial_rotation_M, rotation_M, x_dot_, 2.0, time.toSec(), approach_start_time_);
  }   

  fprintf(save_data_x, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_sensing_(0), f_sensing_(1), f_sensing_(2), f_sensing_(3), f_sensing_(4), f_sensing_(5));
}

void TorqueJointSpaceControllerSideChair::revolve(const ros::Time& time, 
  const Eigen::Vector3d axis,
  const Eigen::Vector3d position, 
  const Eigen::Matrix3d rotation_M, 
  const Eigen::Matrix<double, 6, 1> x_dot_,
  const double range,
  const double duration)
{
  bool is_done;
  double play_time;
  double ang_vel;

  ang_vel = range/duration;

  if(is_revolve_first_ == true)
  {
    revolve_origin_ = position;
    initial_rotation_M = rotation_M;
    revolve_start_time_ = time.toSec();
    is_revolve_first_ = false;
    is_keep_state_first_ = true;
    index_ = 500;
    clearMoment();
    std::cout<<"Revolve start!"<<std::endl;
  }
  
  play_time = time.toSec() - revolve_start_time_;
    
  if(index_ - SIZE <= SIZE+1)
  {
    getInitialFT(index_-SIZE);
    index_++; 
  }
  
  if(play_time <= duration)
  {
    f_star_ = keepCurrentPosition(revolve_origin_, position, x_dot_);
    f_star_(2) = -25.0;//final_force_;
    m_star_ = rotateWithGivenAxis(axis, initial_rotation_M, rotation_M, x_dot_, ang_vel, range, time.toSec(), revolve_start_time_); 
    getMoment(f_sensing_, f_sensing_ee_);   
  }   
  else if(play_time > duration && play_time <= 2*duration)
  {
    if(revolve_direction_ == 0)
    {
      revolve_origin_ = position;
      initial_rotation_M = rotation_M;
      revolve_direction_ ++;
      // detect_contact_ = checkSideChairDone(mx_,my_,mz_,1.8);
      detect_contact_ = checkSideChairDone(mx_ee_,my_ee_,mz_ee_,1.5);
      if(detect_contact_ == true)
      {
        contact_points_++;
        std::cout<<"CHECK CONTACT"<<std::endl;
      } 
      else std::cout<<"Back to the origin"<<std::endl;
    }

    f_star_ = keepCurrentPosition(revolve_origin_, position, x_dot_);
    f_star_(2) = -25.0;//final_force_;
    m_star_ = rotateWithGivenAxis(axis, initial_rotation_M, rotation_M, x_dot_, -ang_vel, -range, time.toSec(), revolve_start_time_ + duration);
  }
  else if(2*duration < play_time && play_time <= 3*duration)
  {
    if(revolve_direction_ == 1)
    {
      revolve_origin_ = position;
      initial_rotation_M = rotation_M;
      revolve_direction_ ++;
      index_ = 500;
      clearMoment();
      std::cout<<"Keep revolving"<<std::endl;
    }

    f_star_ = keepCurrentPosition(revolve_origin_, position, x_dot_);
    f_star_(2) = -25.0;//final_force_;
    m_star_ = rotateWithGivenAxis(axis, initial_rotation_M, rotation_M, x_dot_, -ang_vel, -range, time.toSec(), revolve_start_time_ + 2*duration);
    getMoment(f_sensing_, f_sensing_ee_);   
  }
  else
  {
    if(revolve_direction_ == 2)
    {
      revolve_origin_ = position;
      initial_rotation_M = rotation_M;
      revolve_direction_ ++;
      // detect_contact_ = checkSideChairDone(mx_,my_,mz_,1.8);
      detect_contact_ = checkSideChairDone(mx_ee_,my_ee_,mz_ee_,1.5);
      if(detect_contact_ == true)
      {
        contact_points_++;
        index_ = 0;
        std::cout<<"CHECK CONTACT"<<std::endl;
      } 
      else std::cout<<"Back to the origin"<<std::endl;
    }
    
    f_star_ = keepCurrentPosition(revolve_origin_, position, x_dot_);
    f_star_(2) = -25.0;//final_force_;
    m_star_ = rotateWithGivenAxis(axis, initial_rotation_M, rotation_M, x_dot_, ang_vel, range, time.toSec(), revolve_start_time_ + 3*duration);
  }
  
  if(play_time > 4*duration && contact_count_ == 3) std::cout<<"ONLY TWO POINT CONTACT"<<std::endl;
}


void TorqueJointSpaceControllerSideChair::keepState(const ros::Time& time, const Eigen::Vector3d position, const Eigen::Matrix3d rotation_M, const Eigen::Matrix<double, 6, 1> x_dot_)
{
  if(is_keep_state_first_ == true)
  {
    keep_state_origin_ = position;
    initial_rotation_M = rotation_M;
    is_keep_state_first_ = false;
    keep_state_start_time_ = time.toSec();    
  }

  else
  {
    f_star_ = keepCurrentPosition(keep_state_origin_, position, x_dot_);
    //m_star_ = keepCurrentState(keep_state_origin_, initial_rotation_M, rotation_M, position, x_dot_).tail<3>();
  }
}

void TorqueJointSpaceControllerSideChair::getInitialFT(const int index)
{
  double temp;
  Eigen::Vector3d temp_m;
  
  temp = f_sensing_(2);
  temp_m << f_sensing_ee_(3),  f_sensing_ee_(4),  f_sensing_ee_(5);
  
  if(index == 0)
  {
    force_sum_ = 0.0;
    moment_sum_.setZero();
  } 

  force_sum_ += temp;

  for(int i = 0; i < 3; i++)
  {
    moment_sum_(i) += temp_m(i); 
  }

  if(index > SIZE)
  {
    initial_force_ = force_sum_/SIZE;
    initial_moment_ = moment_sum_/SIZE;

    std::cout<<"initial_force: "<<initial_force_<<std::endl;
    std::cout<<"initial_moment: "<<initial_moment_.transpose()<<std::endl;
  }
}


void TorqueJointSpaceControllerSideChair::getDirectionVector(const Eigen::Vector3d position, const Eigen::Matrix3d rotation)
{
  nx_.push_back(-rotation(0,2));
  ny_.push_back(-rotation(1,2));
  nz_.push_back(-rotation(2,2));
  ex_.push_back(position(0));
  ey_.push_back(position(1));
  ez_.push_back(position(2));
  p_.push_back(nx_.back()*ex_.back() + ny_.back()*ey_.back() + nz_.back()*ez_.back());

  fprintf(save_dir, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t\n", ex_.back(), ey_.back(), ez_.back(), nx_.back(), ny_.back(), nz_.back(), p_.back());

}

void TorqueJointSpaceControllerSideChair::clearDirectionVector()
{
  nx_.clear();
  ny_.clear();
  nz_.clear();
  ex_.clear();
  ey_.clear();
  ez_.clear();
  p_.clear();
}
void TorqueJointSpaceControllerSideChair::getMoment(const Eigen::Matrix<double, 6, 1> f,
  const Eigen::Matrix<double, 6, 1> f_ee)
{
  mx_.push_back(f(3));
  my_.push_back(f(4));
  mz_.push_back(f(5));
  mx_ee_.push_back(f_ee(3));
  my_ee_.push_back(f_ee(4));
  mz_ee_.push_back(f_ee(5));
  // std::cout<<mx_.back()<<std::endl;
  fprintf(save_result, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", mx_.back(), my_.back(), mz_.back(), mx_ee_.back(), my_ee_.back(), mz_ee_.back());
}

void TorqueJointSpaceControllerSideChair::clearMoment()
{
  mx_.clear();
  my_.clear();
  mz_.clear();
  mx_ee_.clear();
  my_ee_.clear();
  mz_ee_.clear();
}

} // namespace advanced_robotics_franka_controllers


PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerSideChair,
                       controller_interface::ControllerBase)
