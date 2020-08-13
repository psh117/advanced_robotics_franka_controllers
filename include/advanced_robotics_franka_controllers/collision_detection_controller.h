#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <franka/robot_state.h>
#include <memory>
#include <random>
#include <fstream>
#include <ftd2xx.h>
#include <pthread.h>
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


namespace advanced_robotics_franka_controllers
{
    class CollisionDetectionController: public controller_interface::MultiInterfaceController<
        franka_hw::FrankaModelInterface, 
        hardware_interface::EffortJointInterface, 
        franka_hw::FrankaStateInterface>
    {
    public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time& time) override;
        void update(const ros::Time& time, const ros::Duration& period) override;
        virtual ~CollisionDetectionController();

    private:
        void generateNewMotion(void);
        bool checkForNewMotion(void);
        void publish(void);
        void generate(void);
        void CollisionDetectionController::wait(double current_time, Eigen::Matrix<double, 7, 1>& q_desired, 
            Eigen::Matrix<double, 7, 1>& qd_desired, Eigen::Map<const Eigen::Matrix<double, 7, 1>>& q);
        void CollisionDetectionController::exec(double current_time, Eigen::Matrix<double, 7, 1>& q_desired, 
            Eigen::Matrix<double, 7, 1>& qd_desired);
        void CollisionDetectionController::rest(double current_time, Eigen::Matrix<double, 7, 1>& q_desired, 
            Eigen::Matrix<double, 7, 1>& qd_desired);
        static Eigen::Vector3d quintic_spline(
            double time,       // Current time
            double time_0,     // Start time
            double time_f,     // End time
            double x_0,        // Start state
            double x_dot_0,    // Start state dot
            double x_ddot_0,   // Start state ddot
            double x_f,        // End state
            double x_dot_f,    // End state
            double x_ddot_f    // End state ddot
        );
        
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        franka_hw::TriggerRate print_rate_trigger_{10}; 

        std::random_device rd;
        std::default_random_engine generator;
        std::uniform_real_distribution<double> angles[7];

        const double joint_q_max[7] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
        const double joint_q_min[7] = {-2.8973, -1.7628, -2.8973, -3.0178, -2.8973, -0.0175, -2.8973};

        moveit::planning_interface::MoveGroupInterface::Plan random_plan;
        moveit::planning_interface::MoveGroupInterface::Plan safe_random_plan;
        franka::RobotState robot_state;
        Eigen::Matrix<double, 7, 1> tau_c;
        Eigen::Matrix<double, 7, 1> tau_dyn;
        pthread_mutex_t mutex;
        FT_HANDLE ft_handle;
        bool ft232h;
        std::ofstream fs;
        int mode;
        bool initialized;
        bool generate_random_motion;
        bool random_motion_generated;
        bool waiting, executing, resting;
        double start_time, end_time;
        int waypoint, waypoints;
        double global_start_time;
    };    
}  // namespace advanced_robotics_franka_controllers
