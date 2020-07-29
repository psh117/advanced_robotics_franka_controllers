
#include <advanced_robotics_franka_controllers/jaesug_controller.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include "math_type_define.h"

namespace advanced_robotics_franka_controllers
{

  bool JaesugController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::vector<std::string> joint_names;
    std::string arm_id;
    ROS_WARN(
        "ForceExampleController: Make sure your robot's endeffector is in contact "
        "with a horizontal surface before starting the controller!");
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
      return false;
    }
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR(
          "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
          "controller init!");
      return false;
    }

    auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "ForceExampleController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "ForceExampleController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      try
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    robot_ = new RobotModel();
    robot_test = new RobotModel();

    save_data = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/JS_data/save_data.txt", "w");
    save_data2 = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/JS_data/save_data2.txt", "w");
    save_data3 = fopen("/home/dyros/catkin_ws/src/advanced_robotics_franka_controllers/JS_data/save_data3.txt", "w");

    return true;
  }

  void JaesugController::starting(const ros::Time &time)
  {
    start_time_ = time;

    for (size_t i = 0; i < 7; ++i)
    {
      q_init_(i) = joint_handles_[i].getPosition();
    }
    //q_desired = q_init_;

    const franka::RobotState &robot_state = state_handle_->getRobotState();
    transform_init_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
  }

  void JaesugController::update(const ros::Time &time, const ros::Duration &period)
  {

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
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Matrix<double, 3, 3> rotation_M(transform.rotation());
    mass_inv = mass_matrix.inverse();

    LAMBDA.resize(6, 6);
    LAMBDA.setZero();

    int ctrl_mode = 1;

    ////////////// LPF/////////////////
    double cutoff = 20.0; // Hz //20
    double RC = 1.0 / (cutoff * 2.0 * M_PI);
    double dt = 0.001;
    double alpha = dt / (RC + dt);
    for (size_t i = 0; i < 7; i++)
    {
      dq_filtered_(i) = alpha * robot_state.dq[i] + (1 - alpha) * dq_filtered_(i);
    }
    robot_->getUpdateKinematics(q, dq_filtered_);
    //////////////////////////////////////
    tip1 << 0.0, 0.0, -0.22;  //for position control
    tip2 << 0.0, 0.6, -0.22;  //for ori1
    tip3 << -0.6, 0.0, -0.22; //for ori2
    tipVector1 = tip2 - tip1;
    tipVector2 = tip3 - tip1;
    pos_ee = robot_->getPosition(7, tip1);
    pos_virtual1 = robot_->getPosition(7, tip2);
    pos_virtual2 = robot_->getPosition(7, tip3);
    Eigen::Vector3d ZZ;
    Eigen::Vector3d to7;
    ZZ.setZero();
    to7 << 0.088, 0.0, -0.22;
    pos_wrist = robot_->getPosition(6, ZZ);

    Rot_cur = robot_->getOrientation(7);
    Rot_wrist = robot_->getOrientation(6);
    angle = DyrosMath::rot2Euler(Rot_cur);
    angle_wrist = DyrosMath::rot2Euler(Rot_wrist);
    J_task.resize(6, 7);

    Jacob_ee.resize(6, 7);
    Jacob_wrist.resize(6, 7);
    Jacob_ee2.resize(6, 7);
    Jacob_ee3.resize(6, 7);
    Jacob_ee = robot_->getJacobian(7, tip1);
    Jacob_wrist = robot_->getJacobian(6, ZZ);
    Jacob_ee2 = robot_->getJacobian(7, tip2);
    Jacob_ee3 = robot_->getJacobian(7, tip3);

    pos_ee_dot.resize(6);
    pos_virtual1_dot.resize(6);
    pos_virtual2_dot.resize(6);
    pos_wrist_dot.resize(6);
    pos_ee_dot = Jacob_ee * dq_filtered_;
    pos_wrist_dot = Jacob_wrist * dq_filtered_;
    pos_virtual1_dot = Jacob_ee2 * dq_filtered_;
    pos_virtual2_dot = Jacob_ee3 * dq_filtered_;

    //q_goal.setZero();
    //q_goal << 0, 0, 0, -M_PI / 2, 0, M_PI / 2, 0;
    q_goal << 0, -0.89, 0, -2.245, 0, 1.46, 0;
    //q_desired.setZero();

    ros::Duration simulation_time = time - start_time_;
    Eigen::Matrix<double, 7, 1> tau_cmd;
    tau_cmd.setZero();

    double trajectory_time = 1.0;

    if (time.toSec() >= start_time_.toSec() && time.toSec() <= (start_time_.toSec() + 3.0))
    {
      for (int i = 0; i < 7; i++)
      {
        q_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec(), start_time_.toSec() + 3.0,
                                        q_init_(i), q_goal(i), 0.0, 0.0);
        qd_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec(), start_time_.toSec() + 3.0,
                                            q_init_(i), q_goal(i), 0.0, 0.0);
      }
      tau_cmd = (200.0 * (q_desired - q) + 7.0 * (qd_desired - dq_filtered_)); // + coriolis;
    }
    else if (time.toSec() > (start_time_.toSec() + 3.0) && time.toSec() < (start_time_.toSec() + 3.01))
    {
      q_init_ = q;
      q_desired = q_init_;
      qd_desired.setZero();
      pos_ee_init = pos_ee;
      pos_wrist_init = pos_wrist;
      pos_virtual1_init = pos_virtual1;
      pos_virtual2_init = pos_virtual2;
      Rot_init = Rot_cur;
      Rot_wrist_init = Rot_cur;
      pos_ee_desired = pos_ee_init;
      pos_ee_desired_pre = pos_ee_desired;
      pos_wrist_desired = pos_wrist_init;
      pos_ee_dot_desired.setZero();
      pos_wrist_dot_desired.setZero();
      angle_init = DyrosMath::rot2Euler(Rot_init);
      angle_wrist_init = DyrosMath::rot2Euler(Rot_wrist_init);
      angle_desired = angle_init;
      angle_dot_desired.setZero();
      pos_ee_goal(0) = pos_ee_init(0) + 0.06;
      pos_ee_goal(1) = pos_ee_init(1) - 0.06;
      pos_ee_goal(2) = pos_ee_init(2) - 0.06;

      //angle_goal(0) = angle_init(0) + 10.0*DEG2RAD;
      //angle_goal(1) = angle_init(1) + 10.0*DEG2RAD;
      angle_goal(0) = angle_wrist_init(0) + 10.0 * DEG2RAD;
      angle_goal(1) = angle_wrist_init(1) + 10.0 * DEG2RAD;
      angle_goal(2) = angle_wrist_init(2) + 0.0 * DEG2RAD;

      pos_wrist_goal(0) = pos_wrist_init(0) + 0.06;
      pos_wrist_goal(1) = pos_wrist_init(1) - 0.06;
      pos_wrist_goal(2) = pos_wrist_init(2) - 0.06;
      Rot_goal = DyrosMath::Euler2rot(angle_goal);
      Rot_i2g = Rot_init.transpose() * Rot_goal;
      axis_angle_goal = acos((Rot_i2g(0, 0) + Rot_i2g(1, 1) + Rot_i2g(2, 2) - 1.0) / 2.0);
      tau_cmd.setZero();
    }
    else if (time.toSec() >= (start_time_.toSec() + 3.01) && time.toSec() <= (start_time_.toSec() + 3.01 + trajectory_time + 1.0))
    {
      if (time.toSec() >= (start_time_.toSec() + 3.01) && time.toSec() <= (start_time_.toSec() + 3.01 + trajectory_time))
      {
        for (int i = 0; i < 3; ++i)
        {
          pos_ee_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec() + 3.01, start_time_.toSec() + 3.01 + trajectory_time,
                                               pos_ee_init(i), pos_ee_goal(i), 0.0, 0.0);
          pos_ee_dot_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec() + 3.01, start_time_.toSec() + 3.01 + trajectory_time,
                                                      pos_ee_init(i), pos_ee_goal(i), 0.0, 0.0);
          // angle_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec() + 3.01, start_time_.toSec() + 3.01 + trajectory_time, //wrist control
          //                                     angle_wrist_init(i), angle_goal(i), 0.0, 0.0);
          // angle_dot_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec() + 3.01, start_time_.toSec() + 3.01 + trajectory_time,
          //                                     angle_wrist_init(i), angle_goal(i), 0.0, 0.0);
          angle_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec() + 3.01, start_time_.toSec() + 3.01 + trajectory_time, //end effector
                                              angle_init(i), angle_goal(i), 0.0, 0.0);
          angle_dot_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec() + 3.01, start_time_.toSec() + 3.01 + trajectory_time,
                                                     angle_init(i), angle_goal(i), 0.0, 0.0);
          pos_wrist_desired(i) = DyrosMath::cubic(time.toSec(), start_time_.toSec() + 3.01, start_time_.toSec() + 3.01 + trajectory_time,
                                                  pos_wrist_init(i), pos_wrist_goal(i), 0.0, 0.0);
          pos_wrist_dot_desired(i) = DyrosMath::cubicDot(time.toSec(), start_time_.toSec() + 3.01, start_time_.toSec() + 3.01 + trajectory_time,
                                                         pos_wrist_init(i), pos_wrist_goal(i), 0.0, 0.0);
        }

        if (axis_angle_goal != 0)
        {
          axis_angle_vector_goal(0) = (Rot_i2g(2, 1) - Rot_i2g(1, 2)) / (2 * sin(axis_angle_goal));
          axis_angle_vector_goal(1) = (Rot_i2g(0, 2) - Rot_i2g(2, 0)) / (2 * sin(axis_angle_goal));
          axis_angle_vector_goal(2) = (Rot_i2g(1, 0) - Rot_i2g(0, 1)) / (2 * sin(axis_angle_goal));

          axis_angle_desired = DyrosMath::cubic(time.toSec(), start_time_.toSec() + 3.01, start_time_.toSec() + 3.01 + trajectory_time,
                                                0.0, axis_angle_goal, 0.0, 0.0);

          Rot_desired = DyrosMath::angleaxis2rot(axis_angle_vector_goal, axis_angle_desired);
        }
        else
        {
          Rot_desired.setIdentity();
        }
        Rot_desired = DyrosMath::Euler2rot(angle_desired);
        // pos_ee_desired = pos_wrist_desired + Rot_desired*to7; // wrist control
        // pos_ee_dot_desired.setZero();
        // pos_wrist_dot_desired.setZero();

        pos_ee_desired2 = pos_ee_desired + Rot_desired * tipVector1;
        pos_ee_desired3 = pos_ee_desired + Rot_desired * tipVector2;
      }
      else if (time.toSec() > (start_time_.toSec() + 3.01 + trajectory_time))
      {
        // pos_ee_desired = pos_wrist_goal + Rot_desired*to7; //wrist control
        pos_ee_desired = pos_ee_goal;
        pos_ee_dot_desired.setZero();
        angle_desired = angle_goal;
        Rot_desired = Rot_goal;
        axis_angle_desired = axis_angle_goal;
        pos_ee_desired2 = pos_ee_desired + Rot_desired * tipVector1;
        pos_ee_desired3 = pos_ee_desired + Rot_desired * tipVector2;
        pos_wrist_desired = pos_wrist_goal; //os_ee_goal - Rot_goal*to7;
        pos_wrist_dot_desired.setZero();
      }

      if (ctrl_mode == 1)
      { //original
        J_task = Jacob_ee;
        fstar = getfstar();

        tau_cmd = calctasktorque(J_task, fstar);

        
        // hosang
        S_mod.resize(3,7);
        S_mod.setZero();
        S_mod(0,4) = 1.0;
        S_mod(1,5) = 1.0;
        S_mod(2,6) = 1.0;
        
        fstar_mod.resize(3);
        fstar_mod.setZero();
        tau_mod.resize(3);
        tau_mod.setZero();

        J_mod.resize(3,7);
        J_mod.setZero();
        Z.resize(3,3);
        Z.setZero();
        J_mod = J_task.block(3,0,3,7);
        Z = J_mod*mass_inv*S_mod.transpose();
        //cout << q << endl << endl;
        Eigen::Vector3d delphi = -DyrosMath::getPhi(Rot_cur, Rot_desired);

    // double Kpx = 2000.0;
    // double Kvx = 20.0;
    // double Kpr = 900.0;
    // double Kvr = 10.0;

        double Kpr_mod = 36000.0;//12000 //24000 //26000
        double Kvr_mod = 250.0;//30(damp up) //80 //120
        for(int i=0;i<3;i++)
          fstar_mod(i) = Kpr_mod * (delphi(i)) + Kvr_mod * (-pos_ee_dot(i + 3));

        tau_mod = Z.inverse()*fstar_mod;
        //tau_mod.setZero();

        for(int i=0;i<3;i++)
          tau_cmd(i+4) = tau_cmd(i+4) + tau_mod(i);
        //tau_cmd.setZero();
      }
      else if (ctrl_mode == 2)
      {                                                         //3point
        J_task.block(0, 0, 3, 7) = Jacob_ee.block(0, 0, 3, 7);  //position 3
        J_task.block(3, 0, 1, 7) = Jacob_ee2.block(2, 0, 1, 7); //z for ori
        J_task.block(4, 0, 2, 7) = Jacob_ee3.block(1, 0, 2, 7); //y,z for ori
        fstar = getfstar2();

        tau_cmd = calctasktorque(J_task, fstar);
      }
      else if (ctrl_mode == 3)
      { //wrist
        J_task.block(0, 0, 3, 7) = Jacob_wrist.block(0, 0, 3, 7);
        J_task.block(3, 0, 3, 7) = Jacob_wrist.block(3, 0, 3, 7);
        //J_task.block(5,0,1,7) = Jacob_ee.block(5,0,1,7);
        fstar = getfstar3();

        tau_cmd = calctasktorque(J_task, fstar);
      }
      else if (ctrl_mode == 4)
      { //joint space
        J_task = Jacob_ee;
        //dyn_consist_ik(J_task);
        clik(J_task);
        gstar = getgstar(q, dq_filtered_);

        tau_cmd = calcjointtorque(gstar);
      }
      //tau_cmd.setZero();
    }
    else
    {
      tau_cmd.setZero();
    }

    robot_->getUpdateKinematics(q_desired, dq_filtered_);
    Eigen::Vector3d pos_ee_test;
    pos_ee_test = robot_test->getPosition(7, tip1);

    fprintf(save_data, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n",
            (time.toSec() - start_time_.toSec()), angle(0), angle(1), angle(2), angle_desired(0), angle_desired(1), angle_desired(2));
    fprintf(save_data2, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n",
            (time.toSec() - start_time_.toSec()), q(0), q(1), q(2), q(3), q(4), q(5));
    fprintf(save_data3, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t\n",
            (time.toSec() - start_time_.toSec()), q_desired(0), q_desired(1), q_desired(2), q_desired(3), q_desired(4), q_desired(5));

    if (print_rate_trigger_())
    {
      ROS_INFO("--------------------------------------------------");
      //ROS_INFO_STREAM("error_pos :" << (pos_init_ - position).transpose() );
      //ROS_INFO_STREAM("error_ori :" << e_rot.transpose() );
      // ROS_INFO_STREAM("time :"<< simulation_time);
      // ROS_INFO_STREAM("tau :" << tau_cmd.transpose());
      // ROS_INFO_STREAM("lambda :" << LAMBDA);
      // ROS_INFO_STREAM("X_ee :" << pos_ee.transpose());
      // ROS_INFO_STREAM("X_ee2 :" << pos_virtual1.transpose());
      // ROS_INFO_STREAM("X_ee3 :" << pos_virtual2.transpose());
      // ROS_INFO_STREAM("q_curent : "<< q.transpose());
      // ROS_INFO_STREAM("q_desired : "<< q_desired.transpose());
      //ROS_INFO_STREAM("ee_pos_rbdl : "<< pos.transpose());
      //ROS_INFO_STREAM("ee_pos_ros : "<< position.transpose());
    }

    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(tau_cmd(i));
    }

    pos_ee_desired_pre = pos_ee_desired;
    Rot_desired_pre = Rot_desired;
  }

  Eigen::Matrix<double, 6, 1> JaesugController::getfstar()
  { //original fstar
    Eigen::Matrix<double, 6, 1> fstar;
    Eigen::Vector3d delphi = -DyrosMath::getPhi(Rot_cur, Rot_desired);
    double Kpx = 400.0;
    double Kvx = 30.0;
    double Kpr = 900.0;//10000
    double Kvr = 60.0;//60

    for (int i = 0; i < 3; ++i)
    {
      fstar(i) = Kpx * (pos_ee_desired(i) - pos_ee(i)) + Kvx * (pos_ee_dot_desired(i) - pos_ee_dot(i));
      fstar(i + 3) = Kpr * (delphi(i)) + Kvr * (-pos_ee_dot(i + 3));
    }

    return fstar;
  }

  Eigen::Matrix<double, 6, 1> JaesugController::getfstar2()
  { //3point fstar
    Eigen::Matrix<double, 6, 1> fstar;
    double Kp = 500.0;
    double Kv = 40.0;
    double Kpr = 100.0;//5000 50
    double Kvr = 2.0;

    for (int i = 0; i < 3; ++i)
    {
      fstar(i) = Kp * (pos_ee_desired(i) - pos_ee(i)) + Kv * (pos_ee_dot_desired(i) - pos_ee_dot(i));
    }
    fstar(3) = Kpr * (pos_ee_desired2(2) - pos_virtual1(2)) + Kvr * (-pos_virtual1_dot(2)); //2-z
    fstar(4) = Kpr * (pos_ee_desired3(1) - pos_virtual2(1)) + Kvr * (-pos_virtual2_dot(1)); //3-y
    fstar(5) = Kpr * (pos_ee_desired3(2) - pos_virtual2(2)) + Kvr * (-pos_virtual2_dot(2)); //3-z

    return fstar;
  }

  Eigen::Matrix<double, 6, 1> JaesugController::getfstar3()
  { //wrist fstar
    Eigen::Matrix<double, 6, 1> fstar;
    Eigen::Vector3d delphi = -DyrosMath::getPhi(Rot_wrist, Rot_desired);
    // Eigen::Vector3d delphi = -DyrosMath::getPhi(Rot_cur,Rot_desired);
    double Kpx = 2000.0;
    double Kvx = 20.0;
    double Kpr = 900.0;
    double Kvr = 10.0;

    for (int i = 0; i < 3; ++i)
    {
      fstar(i) = Kpx * (pos_wrist_desired(i) - pos_wrist(i)) + Kvx * (pos_wrist_dot_desired(i) - pos_wrist_dot(i));
      fstar(i + 3) = Kpr * (delphi(i)) + Kvr * (-pos_wrist_dot(i + 3));
    }

    return fstar;
  }

  Eigen::Matrix<double, 7, 1> JaesugController::getgstar(Eigen::Matrix<double, 7, 1> q, Eigen::Matrix<double, 7, 1> qd)
  { //wrist fstar
    Eigen::Matrix<double, 7, 1> gstar;

    double Kpq = 400.0;
    double Kvq = 40.0;

    for (int i = 0; i < 7; ++i)
    {
      gstar(i) = Kpq * (q_desired(i) - q(i)) + Kvq * (qd_desired(i) - qd(i));
    }
    gstar(3) = 300.0 * (q_desired(3) - q(3)) + 7.0 * (qd_desired(3) - qd(3));
    gstar(4) = 300.0 * (q_desired(4) - q(4)) + 7.0 * (qd_desired(4) - qd(4));
    gstar(5) = 300.0 * (q_desired(5) - q(5)) + 7.0 * (qd_desired(5) - qd(5));
    gstar(6) = 50.0 * (q_desired(6) - q(6)) + 4.0 * (qd_desired(6) - qd(6));

    return gstar;
  }

  Eigen::Matrix<double, 7, 1> JaesugController::calctasktorque(Eigen::Matrix<double, 6, 7> Jtask, Eigen::Matrix<double, 6, 1> fstar)
  {
    Eigen::Matrix<double, 7, 1> torque;
    Eigen::MatrixXd Lambda_inv;
    Eigen::MatrixXd J_T;

    Lambda_inv.resize(6, 6);
    LAMBDA.resize(6, 6);
    J_T.resize(7, 6);
    J_T = Jtask.transpose();

    Lambda_inv = Jtask * mass_inv * J_T;
    LAMBDA = Lambda_inv.inverse();
    torque = J_T * LAMBDA * fstar;

    return torque;
  }

  Eigen::Matrix<double, 7, 1> JaesugController::calcjointtorque(Eigen::Matrix<double, 7, 1> gstar)
  {
    Eigen::Matrix<double, 7, 1> torque;

    torque = gstar;

    return torque;
  }

  void JaesugController::clik(Eigen::Matrix<double, 6, 7> Jtask)
  {
    double kp, kd, kp_ori;

    kp = 1.0;
    kp_ori = 1.0;

    Eigen::Vector6d xd_desired;
    xd_desired.head<3>() = kp * (pos_ee_desired - pos_ee_desired_pre) + pos_ee_dot_desired;

    Eigen::Vector3d delphi = -DyrosMath::getPhi(Rot_desired_pre, Rot_desired);
    xd_desired.tail<3>() = kp_ori * delphi + angle_dot_desired;
    //cout<<xd_desired<<endl<<endl;
    qd_desired = Jtask.transpose() * (Jtask * Jtask.transpose() + Eigen::Matrix6d::Identity() * 0.001).inverse() * xd_desired;
    q_desired = q_desired + qd_desired * 0.001;
  }

  void JaesugController::dyn_consist_ik(Eigen::Matrix<double, 6, 7> Jtask)
  {
    Eigen::Matrix<double, 7, 1> torque;
    Eigen::MatrixXd Lambda_inv;
    Eigen::MatrixXd J_T;

    Lambda_inv.resize(6, 6);
    LAMBDA.resize(6, 6);
    J_T.resize(7, 6);
    J_T = Jtask.transpose();

    Lambda_inv = Jtask * mass_inv * J_T;
    LAMBDA = Lambda_inv.inverse();

    qd_desired = mass_inv * J_T * LAMBDA * pos_ee_dot_desired;
    q_desired = q_desired + qd_desired * 0.001;
  }

} // namespace advanced_robotics_franka_controllers

PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::JaesugController,
                       controller_interface::ControllerBase)
