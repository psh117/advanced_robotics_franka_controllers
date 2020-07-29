#include <advanced_robotics_franka_controllers/torque_joint_space_controller_place.h>
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

bool TorqueJointSpaceControllerPlace::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
  // //joint0_data = fopen("/home/dyros/catkin_ws/src/dyros_mobile_manipulator_controller/joint0_data.txt","w");
  // save_data_x = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_data_fm.txt","w");   
  // save_data_x2 = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_data_pv.txt","w");
  // save_result = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_result.txt","w");
  // save_dir = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_dir.txt","w");
  // save_cmd = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/experiment_data/LEE_spiral/save_cmd.txt","w");
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

void TorqueJointSpaceControllerPlace::starting(const ros::Time& time) {
  
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

  init_origin_.setZero();
  approach_origin_.setZero();
  keep_state_origin_.setZero();
  revolve_origin_.setZero();

  initial_rotation_M.setZero();
  last_rotation_M.setZero();

  rotation_dir_.setZero();
  
  init_config_start_time_ = 0.0;
  approach_start_time_ = 0.0;
  revolve_start_time_ = 0.0;
  keep_state_start_time_ = 0.0;

  detect_contact_ = false;
  rotation_done_ = false;

  is_config_first_ = true;
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

  ee_z_axis_.setZero();
  normal_vector_.setZero();
  rotation_axis_.setZero();
  p1_.setZero(); // 0.527, 0.160, 0.515
  p2_.setZero(); // 0.554, -0.208, 0.447
  p3_.setZero(); // 0.381, -0.235, 0.445
  //p4 --> 0.095, -0.306, 0.431
  //target x,y --> 0.539, -0.001
  angle_difference_ = 0.0;

  p1_ << 0.527, 0.160, 0.515;
  p2_ << 0.554, -0.208, 0.447;
  //p3_ << 0.381, -0.235, 0.445;
  p3_ << 0.095, -0.306, 0.431;
}
//0.681, 0.732, -0.000, 0.002
//0.674, 0.734, 0.056, -0.66
void TorqueJointSpaceControllerPlace::update(const ros::Time& time, const ros::Duration& period) {

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
    initConfig(time, position, rotation_M, x_dot_);
  }

  else
  {
    if(index_<=SIZE+1)
    {
      keepState(time, position, rotation_M, x_dot_);
      m_star_.setZero();
      getInitialFT(index_);
      index_++;              
    }

    if(rotation_done_ == false)
    {
      alignAxis(time, position, rotation_M, x_dot_, 15.0);
    }
    else
    {
      approach(time, position, rotation_M, x_dot_);
      
      // std::cout<<"f_star_: "<<f_star_.transpose()<<std::endl;
      // keepState(time, position, rotation_M, x_dot_);
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

  // fprintf(save_data_x2, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", position(0), position(1), position(2), x_dot_(0), x_dot_(1), x_dot_(2), x_dot_(3), x_dot_(4), x_dot_(5));
  // fprintf(save_cmd, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_star_zero_(0), f_star_zero_(1), f_star_zero_(2), f_star_zero_(3), f_star_zero_(4), f_star_zero_(5));

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_cmd(i));
    //joint_handles_[i].setCommand(0);
  }

}

void TorqueJointSpaceControllerPlace::initConfig(const ros::Time& time, 
  const Eigen::Vector3d position, 
  const Eigen::Matrix3d rotation_M, 
  const Eigen::Matrix<double, 6, 1> x_dot_)
{
  if(is_config_first_ == true)
  {
    init_origin_ = position;
    initial_rotation_M = rotation_M;
    init_config_start_time_ = time.toSec();
    is_config_first_ = false;
    is_keep_state_first_ = true; 
    std::cout<<"Init Configuration"<<std::endl;   
  }
  
  else
  {
    keepState(time, position, rotation_M, x_dot_);
    m_star_ = keepOrientationPerpenticularOnlyXY(initial_rotation_M, rotation_M, x_dot_, 2.0, time.toSec(), init_config_start_time_);  
  }   
}

void TorqueJointSpaceControllerPlace::approach(const ros::Time& time, 
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
    std::cout<<"Approach start"<<std::endl;   
  }
  
  else
  {
    f_star_ = straightApproach(approach_origin_, initial_rotation_M, rotation_M, position, x_dot_, time.toSec(), approach_start_time_ ).head<3>();
    m_star_ = keepCurrentOrientation(initial_rotation_M, rotation_M, x_dot_);    
  }   

  // fprintf(save_data_x, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n", f_sensing_(0), f_sensing_(1), f_sensing_(2), f_sensing_(3), f_sensing_(4), f_sensing_(5));
}

void TorqueJointSpaceControllerPlace::alignAxis(const ros::Time& time, 
  const Eigen::Vector3d position, 
  const Eigen::Matrix3d rotation_M, 
  const Eigen::Matrix<double, 6, 1> x_dot_,
  const double duration)
{
  double ang_vel;
  double a,b,c,d;
  Eigen::Vector3d ex,ey,ez;
  Eigen::Vector3d ex_proj,ey_proj;
  Eigen::Matrix3d o_rot_t;
  Eigen::Matrix3d o_rot_t_sol;
  Eigen::Matrix3d e_rot_t;
  Eigen::Matrix3d e_rot_t_sol;
  Eigen::Matrix3d target_rotation_M;
  Eigen::Vector3d euler_angle;
  Eigen::Vector3d delphi_delta;
  double roll, pitch, yaw;
  double alpha, beta, gamma;
  // -0.0090053,  0.9996858,  0.0233918;
  //  0.9867422,  0.0050930,  0.1622157;
  //  0.1620456,  0.0245425, -0.9864780 o_rot_t
  if(is_revolve_first_ == true)
  {
    revolve_origin_ = position;
    initial_rotation_M = rotation_M;
    revolve_start_time_ = time.toSec();
    is_revolve_first_ = false;
    is_keep_state_first_ = true;

    ee_z_axis_ << initial_rotation_M(0,2), initial_rotation_M(1,2), initial_rotation_M(2,2);    
    normal_vector_ = computeNomalVector(p1_, p2_, p3_);

    if(ee_z_axis_(2)*normal_vector_(2) < 0) normal_vector_ = - normal_vector_;
        
    // std::cout<<"normal_vector_: "<<normal_vector_.transpose()<<std::endl;
    // std::cout<<"ee_z_axis_: "<<ee_z_axis_.transpose()<<std::endl;
    // std::cout<<"revolve_origin_: "<<revolve_origin_.transpose()<<std::endl;
    std::cout<<"Revolve start!"<<std::endl;
  }
  
  if(time.toSec() - revolve_start_time_ > duration)
  {
    rotation_done_ = true;
  } 

  a = normal_vector_(0);
  b = normal_vector_(1);
  c = normal_vector_(2);
  d = -a*revolve_origin_(0) - b*revolve_origin_(1) - c*revolve_origin_(2);
  
  ex = initial_rotation_M.block<3,1>(0,0);
  ey = initial_rotation_M.block<3,1>(0,1);
  ez = initial_rotation_M.block<3,1>(0,2);
  
  ex_proj << ex(0), ex(1), (-a*ex(0)-b*ex(1)-d)/c - revolve_origin_(2);
  ey_proj << ey(0), ey(1), (-a*ey(0)-b*ey(1)-d)/c - revolve_origin_(2);

  ex_proj = vectorNormalization(ex_proj);
  ey_proj = vectorNormalization(ey_proj);

  o_rot_t.block<3,1>(0,0) = ex_proj;
  o_rot_t.block<3,1>(0,1) = ey_proj;
  o_rot_t.block<3,1>(0,2) = normal_vector_;

  o_rot_t_sol << 0.0090053, 0.9996858, 0.0233918, 0.9867422, 0.0050930, 0.1622157, 0.1620456, 0.0245425, -0.9864780; 

  e_rot_t = initial_rotation_M.transpose()*o_rot_t;
  e_rot_t_sol = initial_rotation_M.transpose()*o_rot_t_sol;
  
  
  euler_angle = DyrosMath::rot2Euler(e_rot_t); //w.r.p end-effector frame
  roll = euler_angle(0);
  pitch = euler_angle(1);
  yaw = euler_angle(2);
  
  alpha = DyrosMath::cubic(time.toSec(), revolve_start_time_, revolve_start_time_ + duration, 0, roll, 0, 0);
  beta = DyrosMath::cubic(time.toSec(), revolve_start_time_, revolve_start_time_ + duration, 0, pitch, 0, 0);
  gamma = DyrosMath::cubic(time.toSec(), revolve_start_time_, revolve_start_time_ + duration, 0, yaw, 0, 0);


  target_rotation_M = initial_rotation_M*DyrosMath::rotateWithZ(gamma) * DyrosMath::rotateWithY(beta) * DyrosMath::rotateWithX(alpha);
  // target_rotation_M = DyrosMath::rotateWithX(alpha) * DyrosMath::rotateWithY(beta) * DyrosMath::rotateWithZ(yaw)*initial_rotation_M;

  delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);

  f_star_ = keepCurrentPosition(revolve_origin_, position, x_dot_);
  m_star_ = (1.0) * 200.0 * delphi_delta + 5.0 * (-x_dot_.tail<3>());  
  
}


void TorqueJointSpaceControllerPlace::keepState(const ros::Time& time, const Eigen::Vector3d position, const Eigen::Matrix3d rotation_M, const Eigen::Matrix<double, 6, 1> x_dot_)
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

void TorqueJointSpaceControllerPlace::getInitialFT(const int index)
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


void TorqueJointSpaceControllerPlace::getDirectionVector(const Eigen::Vector3d position, const Eigen::Matrix3d rotation)
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

void TorqueJointSpaceControllerPlace::clearDirectionVector()
{
  nx_.clear();
  ny_.clear();
  nz_.clear();
  ex_.clear();
  ey_.clear();
  ez_.clear();
  p_.clear();
}
void TorqueJointSpaceControllerPlace::getMoment(const Eigen::Matrix<double, 6, 1> f,
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

void TorqueJointSpaceControllerPlace::clearMoment()
{
  mx_.clear();
  my_.clear();
  mz_.clear();
  mx_ee_.clear();
  my_ee_.clear();
  mz_ee_.clear();
}

} // namespace advanced_robotics_franka_controllers


PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::TorqueJointSpaceControllerPlace,
                       controller_interface::ControllerBase)
