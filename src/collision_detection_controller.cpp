#include <advanced_robotics_franka_controllers/collision_detection_controller.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ftd2xx.h>
#include <pthread.h>
#include <random>
#include <fstream>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <unistd.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot_state.h>
#include "math_type_define.h"

#define MAX_FILE_SIZE 100000000 // 100MB


namespace advanced_robotics_franka_controllers
{
    enum MODE {WAIT, EXEC, REST};

    bool CollisionDetectionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
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
        if ((!node_handle.getParam("joint_names", joint_names)) || (joint_names.size() != 7))
        {
            ROS_ERROR(
                "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
                "controller init!");
            return false;
        }

        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
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
        catch (hardware_interface::HardwareInterfaceException& ex)
        {
            ROS_ERROR_STREAM(
                "ForceExampleController: Exception getting model handle from interface: " << ex.what());
            return false;
        }

        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
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
        catch (hardware_interface::HardwareInterfaceException& ex)
        {
            ROS_ERROR_STREAM(
                "ForceExampleController: Exception getting state handle from interface: " << ex.what());
            return false;
        }

        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; i++)
        {
            try
            {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException& ex)
            {
                ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        FT_STATUS ft_status;
        DWORD num_devs;
        DWORD num_bytes_to_send = 0;
        DWORD num_bytes_sent = 0;
        DWORD num_bytes_to_read = 0;
        DWORD num_bytes_read = 0;
        BYTE buffer[8];
        try
        {
            if ((ft_status = FT_CreateDeviceInfoList(&num_devs)) != FT_OK) throw 1;
            if (num_devs < 1) { ROS_ERROR_STREAM("There are no FTDI devices installed"); return false; }
            if ((ft_status = FT_Open(0, &ft_handle)) != FT_OK) throw 2;
            if ((ft_status = FT_ResetDevice(ft_handle)) != FT_OK) throw 3;
            if ((ft_status = FT_GetQueueStatus(ft_handle, &num_bytes_to_read)) != FT_OK) throw 4;
            if (num_bytes_to_read > 0) FT_Read(ft_handle, &buffer, num_bytes_to_read, &num_bytes_read);
            if ((ft_status = FT_SetLatencyTimer(ft_handle, 2)) != FT_OK) throw 5;
            if ((ft_status = FT_SetChars(ft_handle, false, 0, false, 0)) != FT_OK) throw 6;
            if ((ft_status = FT_SetTimeouts(ft_handle, 7, 1000)) != FT_OK) throw 7;
            if ((ft_status = FT_SetFlowControl(ft_handle, FT_FLOW_NONE, 0x0, 0x0)) != FT_OK) throw 8;
            if ((ft_status = FT_SetBitMode(ft_handle, 0x0, FT_BITMODE_RESET)) != FT_OK) throw 9;
            if ((ft_status = FT_SetBitMode(ft_handle, 0x0, FT_BITMODE_MPSSE)) != FT_OK) throw 10;
            usleep(50000);
            // Configure data bits high-byte of MPSSE port
            buffer[0] = 0x82; buffer[1] = 0x00; buffer[2] = 0x00; num_bytes_to_send = 3;
            // Send off the high GPIO config commands
            if ((ft_status = FT_Write(ft_handle, buffer, num_bytes_to_send, &num_bytes_sent)) != FT_OK) throw 11;
            ft232h = true;
        }
        catch (const int step)
        {
            ROS_ERROR_STREAM("MPSSE initialization failed at step " << step);
            if (step > 2) FT_Close(ft_handle);
            ft232h = false;
            return false;
        }

        fs.open("/home/dyros/log1.csv", (std::ofstream::out | std::ofstream::trunc));
        if (!fs.is_open())
        {
            ROS_ERROR_STREAM("Can't open a file for logging");
            FT_Close(ft_handle);
            return false;
        }

        pthread_mutex_init(&mutex, NULL);
        mode = WAIT;
        generate_random_motion = true;
        random_motion_generated = false;
        waiting = false;
        executing = false;
        resting = false;
        start_time = 0.0;
        end_time = 0.0;
        waypoint = 0;
        waypoints = 0;

        thread1 = std::thread(&CollisionDetectionController::generate, this);
        thread2 = std::thread(&CollisionDetectionController::publish, this);

        return true;
    }

    void CollisionDetectionController::starting(const ros::Time& time)
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();
        
        generator.seed(rd());
        double range = 0.0;
        for (size_t i = 0; i < 7; i++)
        {
            range = ((joint_q_max[i] - joint_q_min[i]) * 0.1);
            angles[i] = std::uniform_real_distribution<double>((joint_q_min[i] + range), (joint_q_max[i] - range));
        }
        global_start_time = time.toSec();
    }

    void CollisionDetectionController::generate(void)
    {
        static const std::string PLANNING_GROUP = "arm";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const robot_state::JointModelGroup* joint_model_group = 
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
        visual_tools.deleteAllMarkers();
        visual_tools.loadRemoteControl();

        // Create collision object
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        
        // Box 1
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 1.93;
        primitive.dimensions[1] = 4.0;
        primitive.dimensions[2] = 0.01;
        moveit_msgs::CollisionObject box1;
        box1.header.frame_id = move_group.getPlanningFrame();
        box1.id = "box1";
        geometry_msgs::Pose box1_pose;
        box1_pose.position.x = 1.035;
        box1_pose.position.y = 0.0;
        box1_pose.position.z = 0.095;
        box1.primitives.push_back(primitive);
        box1.primitive_poses.push_back(box1_pose);
        box1.operation = box1.ADD;
        collision_objects.push_back(box1);

        // Box 2
        primitive.dimensions[0] = 1.89;
        moveit_msgs::CollisionObject box2;
        box2.header.frame_id = move_group.getPlanningFrame();
        box2.id = "box2";
        geometry_msgs::Pose box2_pose;
        box2_pose.position.x = -1.055;
        box2_pose.position.y = 0.0;
        box2_pose.position.z = 0.095;
        box2.primitives.push_back(primitive);
        box2.primitive_poses.push_back(box2_pose);
        box2.operation = box2.ADD;
        collision_objects.push_back(box2);

        // Box 3
        primitive.dimensions[0] = 0.18;
        primitive.dimensions[1] = 1.91;
        moveit_msgs::CollisionObject box3;
        box3.header.frame_id = move_group.getPlanningFrame();
        box3.id = "box3";
        geometry_msgs::Pose box3_pose;
        box3_pose.position.x = -0.02;
        box3_pose.position.y = 1.045;
        box3_pose.position.z = 0.095;
        box3.primitives.push_back(primitive);
        box3.primitive_poses.push_back(box3_pose);
        box3.operation = box3.ADD;
        collision_objects.push_back(box3);

        // Box 4
        moveit_msgs::CollisionObject box4;
        box4.header.frame_id = move_group.getPlanningFrame();
        box4.id = "box4";
        geometry_msgs::Pose box4_pose;
        box4_pose.position.x = -0.02;
        box4_pose.position.y = -1.045;
        box4_pose.position.z = 0.095;
        box4.primitives.push_back(primitive);
        box4.primitive_poses.push_back(box4_pose);
        box4.operation = box4.ADD;
        collision_objects.push_back(box4);

        planning_scene_interface.addCollisionObjects(collision_objects);
        visual_tools.trigger();

        std::vector<double> joint_group_positions;
        joint_group_positions.resize(7);
        std::vector<double> q; q.resize(7);
        std::vector<double> dq; dq.resize(7);
        moveit::core::RobotState start_state = *(move_group.getCurrentState());
        move_group.setMaxVelocityScalingFactor(0.2);
        while (1)
        {
            if (!initialized) continue;
            if (!generate_random_motion) continue;
            generate_random_motion = false;

            pthread_mutex_lock(&mutex);
            const franka::RobotState robot_state = this->robot_state;
            pthread_mutex_unlock(&mutex);

            for (int i = 0; i < 7; i++)
            {
                q[i] = robot_state.q[i];
                dq[i] = robot_state.dq[i];
            }
            start_state.setJointGroupPositions(PLANNING_GROUP, q);
            start_state.setJointGroupVelocities(PLANNING_GROUP, dq);
            move_group.setStartState(start_state);
            
            do
            {
                for (int i = 0; i < 7; i++) joint_group_positions[i] = ((angles[i])(generator));
                move_group.setJointValueTarget(joint_group_positions);
            } while (move_group.plan(random_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS);
            safe_random_plan = random_plan;

            random_motion_generated = true;
            visual_tools.publishTrajectoryLine(random_plan.trajectory_, joint_model_group);
            visual_tools.trigger();
        }
    }

    void CollisionDetectionController::publish(void)
    {
        std::chrono::system_clock::time_point time_1;
        std::chrono::system_clock::time_point time_2;
        std::chrono::microseconds interval;
        franka::RobotState robot_state;
        Eigen::Matrix<double, 7, 7> mass_matrix;
        Eigen::Matrix<double, 7, 1> coriolis;
        Eigen::Matrix<double, 7, 1> gravity;
        Eigen::Matrix<double, 7, 1> tau_c;
        Eigen::Matrix<double, 7, 1> tau_dyn;
        DWORD num_bytes;
        BYTE buffer;
        int file_index = 1;
        int collision;
        register int i, j;

        time_1 = (std::chrono::system_clock::now() - std::chrono::milliseconds(11));
        while (1)
        {
            if (!initialized) continue;
            time_2 = std::chrono::system_clock::now();
            interval = std::chrono::duration_cast<std::chrono::microseconds>(time_2 - time_1);
            if (interval.count() < 10000) continue;
            time_1 = std::chrono::system_clock::now();
            if (!ft232h) break;
            if (!fs.is_open()) break;
            buffer = 0x83;
            FT_Write(ft_handle, &buffer, 1, &num_bytes);
            pthread_mutex_lock(&mutex);
            robot_state = this->robot_state;
            mass_matrix = this->mass_matrix;
            coriolis = this->coriolis;
            gravity = this->gravity;
            tau_c = this->tau_c;
            tau_dyn = this->tau_dyn;
            pthread_mutex_unlock(&mutex);
            fs << robot_state.time.toMSec() << ','; // Strictly monotonically increasing timestamp since robot start(in miliseconds)
            for (i = 0; i < 7; i++) fs << robot_state.tau_J[i] << ','; // Measured link-side joint torque sensor signals
            for (i = 0; i < 7; i++) fs << robot_state.dtau_J[i] << ','; // Derivative of measured link-side joint torque sensor signals
            for (i = 0; i < 7; i++) fs << robot_state.q[i] << ','; // Measured joint position
            for (i = 0; i < 7; i++) fs << robot_state.q_d[i] << ','; // Desired joint position
            for (i = 0; i < 7; i++) fs << robot_state.dq[i] << ','; // Measured joint velocity
            for (i = 0; i < 7; i++) fs << robot_state.dq_d[i] << ','; // Desired joint velocity
            for (i = 0; i < 7; i++) fs << robot_state.ddq_d[i] << ','; // Desired joint acceleration
            for (i = 0; i < 7; i++) fs << robot_state.tau_ext_hat_filtered[i] << ','; // Filtered external torque
            for (i = 0; i < 7; i++) fs << robot_state.theta[i] << ','; // Motor position
            for (i = 0; i < 7; i++) fs << robot_state.dtheta[i] << ','; // Motor velocity
            for (i = 0; i < 7; i++) fs << tau_c(i) << ','; // Commanded joint touque
            for (i = 0; i < 7; i++) fs << tau_dyn(i) << ','; // Dynamic torque
            for (i = 0; i < 16; i++) fs << robot_state.O_T_EE[i] << ','; // Measured end effector pose in base frame
            for (i = 0; i < 6; i++) fs << robot_state.O_F_ext_hat_K[i] << ','; // Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame
            for (i = 0; i < 7; i++) { for (j = 0; j < 7; j++) fs << mass_matrix(i, j) << ','; } // Mass matrix
            for (i = 0; i < 7; i++) fs << coriolis(i) << ','; // Coriolis
            for (i = 0; i < 7; i++) fs << gravity(i) << ','; // Gravity
            for (i = 0; i < 7; i++) fs << robot_state.joint_collision[i] << ','; // Indicates which contact level is activated in which joint
            for (i = 0; i < 6; i++) fs << robot_state.cartesian_collision[i] << ','; // Indicates which contact level is activated in which Cartesian dimension
            FT_Read(ft_handle, &buffer, 1, &num_bytes);
            collision = ((buffer & 0x4) > 0) ? 0 : 1;
            fs << collision << ',' << (1 - collision) << std::endl;
            fs.flush();
            if (fs.tellp() > MAX_FILE_SIZE)
            {
                fs.close();
                std::string filename("/home/dyros/log");
                file_index++;
                filename += std::to_string(file_index);
                filename += ".csv";
                fs.open(filename.c_str(), (std::ofstream::out | std::ofstream::trunc));
                if (!fs.is_open())
                {
                    ROS_ERROR_STREAM("Can't open a file for logging");
                    FT_Close(ft_handle);
                    return;
                }
            }
        }
    }

    void CollisionDetectionController::generateNewMotion(void)
    {
        generate_random_motion = true;
    }

    bool CollisionDetectionController::checkForNewMotion(void)
    {
        bool result = random_motion_generated;
        if (result) random_motion_generated = false;
        return result;
    }

    void CollisionDetectionController::update(const ros::Time& time, const ros::Duration& period)
    {
        const franka::RobotState& robot_state = state_handle_->getRobotState();
        const std::array<double, 49>& massmatrix_array = model_handle_->getMass();
        const std::array<double, 7>& coriolis_array = model_handle_->getCoriolis();
        const std::array<double, 7>& gravity_array = model_handle_->getGravity();

        Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass_matrix(massmatrix_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> acceleration(robot_state.ddq_d.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> qd(robot_state.dq.data());

        Eigen::Matrix<double, 7, 1> q_desired;
        Eigen::Matrix<double, 7, 1> qd_desired;
        Eigen::Matrix<double, 7, 1> tau_cmd;

        double current_time = time.toSec();
        double simulation_time = current_time - global_start_time;

        switch (mode)
        {
        case WAIT: wait(current_time, q_desired, qd_desired, q); break;

        case EXEC: exec(current_time, q_desired, qd_desired); break;

        case REST: rest(current_time, q_desired, qd_desired); break;
        }
        
        double kp, kv;
        kp = 1500;
        kv = 10;
        tau_cmd = mass_matrix * (kp * (q_desired - q) + kv * (qd_desired - qd)) + coriolis;

        if (print_rate_trigger_())
        {
            ROS_INFO("--------------------------------------------------");
            ROS_INFO_STREAM("tau: " << tau_cmd.transpose());
            ROS_INFO_STREAM("time :" << simulation_time);
            ROS_INFO_STREAM("q_curent: " << q.transpose());
            ROS_INFO_STREAM("q_desired: " << q_desired.transpose());
        }

        for (int i = 0; i < 7; i++) joint_handles_[i].setCommand(tau_cmd(i));

        pthread_mutex_lock(&mutex);
        this->robot_state = robot_state;
        this->mass_matrix = mass_matrix;
        this->coriolis = coriolis;
        this->gravity = gravity;
        this->tau_dyn = mass_matrix * acceleration + coriolis + gravity;
        this->tau_c = tau_cmd;
        pthread_mutex_unlock(&mutex);
        initialized = true;
    }

    void CollisionDetectionController::wait(const double current_time, Eigen::Matrix<double, 7, 1>& q_desired, 
        Eigen::Matrix<double, 7, 1>& qd_desired, Eigen::Map<const Eigen::Matrix<double, 7, 1>>& q)
    {
        if (!waiting)
        {
            for (int i = 0; i < 7; i++) q_waiting(i) = q(i);
            waiting = true;
        }
        if (checkForNewMotion())
        {
            waiting = false; mode = EXEC;
            exec(current_time, q_desired, qd_desired);
            return;
        }
        for (int i = 0; i < 7; i++)
        {
            q_desired(i) = q_waiting(i);
            qd_desired(i) = 0.0;
        }
    }

    void CollisionDetectionController::exec(const double current_time, Eigen::Matrix<double, 7, 1>& q_desired, 
        Eigen::Matrix<double, 7, 1>& qd_desired)
    {
        double x_0, x_dot_0, x_ddot_0, x_f, x_dot_f, x_ddot_f;
        std::vector<Eigen::Vector3d> cmd;

        if (!executing)
        {
            start_time = current_time;
            waypoints = safe_random_plan.trajectory_.joint_trajectory.points.size();
            waypoint = 0;
            end_time = safe_random_plan.trajectory_.joint_trajectory.points[waypoints - 1].time_from_start.toSec();
            end_time += start_time;
            generateNewMotion();
            executing = true;
        }
        if (current_time > end_time)
        {
            executing = false; mode = REST;
            rest(current_time, q_desired, qd_desired);
            return;
        }
        while (waypoint < waypoints)
        {
            waypoint++;
            if (current_time <= (safe_random_plan.trajectory_.joint_trajectory.points[waypoint].time_from_start.toSec() + 
                start_time)) break; 
        }
        waypoint--;
        double interval_start_time = (safe_random_plan.trajectory_.joint_trajectory.points[waypoint].time_from_start.toSec() + 
            start_time);
        double interval_end_time = (safe_random_plan.trajectory_.joint_trajectory.points[waypoint + 1].time_from_start.toSec() + 
            start_time);
        cmd.resize(7);
        for (int i = 0; i < 7; i++)
        {
            x_0 = safe_random_plan.trajectory_.joint_trajectory.points[waypoint].positions[i];
            x_dot_0 = safe_random_plan.trajectory_.joint_trajectory.points[waypoint].velocities[i];
            x_ddot_0 = safe_random_plan.trajectory_.joint_trajectory.points[waypoint].accelerations[i];
            x_f = safe_random_plan.trajectory_.joint_trajectory.points[waypoint + 1].positions[i];
            x_dot_f = safe_random_plan.trajectory_.joint_trajectory.points[waypoint + 1].velocities[i];
            x_ddot_f = safe_random_plan.trajectory_.joint_trajectory.points[waypoint + 1].accelerations[i];
            cmd[i] = quintic_spline((current_time - interval_start_time), 0, (interval_end_time - interval_start_time), 
                x_0, x_dot_0, x_ddot_0, x_f, x_dot_f, x_ddot_f);
        }
        for (int i = 0; i < 7; i++)
        {
            q_desired(i) = cmd[i](0);
            qd_desired(i) = cmd[i](1);
        }
    }

    void CollisionDetectionController::rest(const double current_time, Eigen::Matrix<double, 7, 1>& q_desired, 
        Eigen::Matrix<double, 7, 1>& qd_desired)
    {
        if (!resting) { resting = true; start_time = current_time; end_time = (start_time + 1.0); }
        if (current_time > end_time) { resting = false; mode = WAIT; }
        for (int i = 0; i < 7; i++)
        {
            q_desired(i) = safe_random_plan.trajectory_.joint_trajectory.points[waypoints - 1].positions[i];
            qd_desired(i) = safe_random_plan.trajectory_.joint_trajectory.points[waypoints - 1].velocities[i];
        }
    }

    Eigen::Vector3d CollisionDetectionController::quintic_spline(
        double time,       // Current time
        double time_0,     // Start time
        double time_f,     // End time
        double x_0,        // Start state
        double x_dot_0,    // Start state dot
        double x_ddot_0,   // Start state ddot
        double x_f,        // End state
        double x_dot_f,    // End state
        double x_ddot_f    // End state ddot
    )
    {
        double a1, a2, a3, a4, a5, a6;
        double time_s;

        Eigen::Vector3d result;

        if (time < time_0)
        {
            result << x_0, x_dot_0, x_ddot_0;
            return result;
        }
        else if (time > time_f)
        {
            result << x_f, x_dot_f, x_ddot_f;
            return result;
        }

        time_s = time_f - time_0;
        a1 = x_0; a2 = x_dot_0; a3 = (x_ddot_0 / 2.0);

        Eigen::Matrix3d Temp;
        Temp << pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
            (3.0 * pow(time_s, 2)), (4.0 * pow(time_s, 3)), (5.0 * pow(time_s, 4)),
            (6.0 * time_s), (12.0 * pow(time_s, 2)), (20.0 * pow(time_s, 3));

        Eigen::Vector3d R_temp;
        R_temp << (x_f - x_0 - x_dot_0 * time_s - x_ddot_0 * pow(time_s, 2) / 2.0),
            (x_dot_f - x_dot_0 - x_ddot_0 * time_s),
            (x_ddot_f - x_ddot_0);

        Eigen::Vector3d RES;

        RES = (Temp.inverse() * R_temp);
        a4 = RES(0); a5 = RES(1); a6 = RES(2);

        double time_fs = (time - time_0);
        double position = (a1 + a2 * pow(time_fs, 1) + a3 * pow(time_fs, 2) + a4 * pow(time_fs, 3) + a5 * pow(time_fs, 4) + a6 * pow(time_fs, 5));
        double velocity = (a2 + 2.0 * a3 * pow(time_fs, 1) + 3.0 * a4 * pow(time_fs, 2) + 4.0 * a5 * pow(time_fs, 3) + 5.0 * a6 * pow(time_fs, 4));
        double acceleration = (2.0 * a3 + 6.0 * a4 * pow(time_fs, 1) + 12.0 * a5 * pow(time_fs, 2) + 20.0 * a6 * pow(time_fs, 3));
        result << position, velocity, acceleration;

        return result;
    }

    CollisionDetectionController::~CollisionDetectionController()
    {
        if (fs.is_open()) fs.close();
        if (ft232h) FT_Close(ft_handle);
    }
} // namespace advanced_robotics_franka_controllers

PLUGINLIB_EXPORT_CLASS(advanced_robotics_franka_controllers::CollisionDetectionController, controller_interface::ControllerBase)