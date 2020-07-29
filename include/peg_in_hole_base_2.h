#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include "math_type_define.h"
#include <cstdlib>
#include <ctime>

using namespace Eigen;

namespace PegInHole2
{

    static Matrix4d setTransformation(const Vector3d position, const Matrix3d rotation)
    {
        Matrix<double, 4, 4> tf;

        tf.block<3,3>(0,0) = rotation;
        tf.col(3) << position(0), position(1), position(2);
        tf.row(3) << 0.0, 0.0, 0.0, 1.0;

        return tf; 
    }
    
    static Matrix4d setTransformation(const Vector3d position, const Vector4d quat)
    {
        Matrix<double, 4, 4> tf;
        Matrix3d rot;
        
        rot = DyrosMath::quat2Rot(quat);
        
        tf.block<3,3>(0,0) = rot;
        tf.col(3) << position(0), position(1), position(2);
        tf.row(3) << 0.0, 0.0, 0.0, 1.0;

        return tf; 
    }


    static Matrix4d setTransformationInverse(const Matrix4d tf)
    {
        Vector3d pos;
        Matrix3d rot;
        Matrix4d tf_inv;

        rot = tf.block<3,3>(0,0);
        pos = tf.block<3,1>(0,3);

        tf_inv.block<3,3>(0,0) = rot.transpose();
        tf_inv.block<3,1>(0,3) = -rot.transpose()*pos;
        tf_inv.block<1,4>(3,0) << 0.0, 0.0, 0.0, 1.0;

        // std::cout<<"rotation_part: \n"<< rot<<std::endl;
        // std::cout<<"position_part: \n"<< pos.transpose()<<std::endl;
        // std::cout<<"-----------------inverse--------------"<<std::endl;
        // std::cout<<"rotation_part: \n"<< rot.transpose()<<std::endl;
        // std::cout<<"rotation_part: \n"<< tf_inv.block<3,3>(0,0)<<std::endl;
        // std::cout<<"position_part: \n"<< tf_inv.block<3,1>(0,3).transpose()<<std::endl;
        // std::cout<<"Inverse_: \n"<<tf_inv<<std::endl;
        return tf_inv;
    }

    static Matrix4d setGrasp2Assembly(const Vector3d position, const Matrix3d rotation, const Matrix4d T_wo, const Matrix4d T_oa)
    {
        Matrix4d T_we, T_we_inv;
        Matrix4d T_ga;

        T_we = setTransformation(position, rotation);
        T_we_inv = setTransformationInverse(T_we);

        T_ga = T_we_inv*T_wo*T_oa;

        return T_ga;
    }

    //z-axis of component 
    static Vector3d getAssemblyDirction(const Matrix4d T_ga)
    {
        Vector3d dir;

        dir = T_ga.block<3,1>(0,2); // because the assembly direction w.r.t the component is always z-axis
        
        return dir;
    }

    static Vector3d getTiltDirection(const Matrix4d T_ga, const Vector3d assembly_dir)
    {   
        //ONLY when the z_e are parallel to the assembly direction!!!
        Vector3d Oa_g; // assembly_point wrt grasp frame {G}
        Vector3d Og_a; // grasp_point wrt assembly frame {A}
        Vector3d N_g; // normal vector wrt grasp frame {G}
        
        Vector3d tilt_axis; //w.r.t assembly_frame, {A}

        Vector3d o1, o2; // candidates {A}
        Matrix3d rot1, rot2; //wrt assembly frame {A}
        Matrix3d R_ga;
        Matrix4d T_ag;
        double theta;

        theta = 10*M_PI/180; //arbitrary selected value to judge which a tilt direction is proper

        R_ga = T_ga.block<3,3>(0,0);
        Oa_g = T_ga.block<3,1>(0,3);

        T_ag = PegInHole2::setTransformationInverse(T_ga);
        Og_a = T_ag.block<3,1>(0,3);


        N_g = Oa_g.cross(assembly_dir);
        N_g = N_g.normalized();
        
        tilt_axis = R_ga.transpose()*N_g;
        
        // rot1 = Eigen::AngleAxisd(theta, tilt_axis).toRotationMatrix();
        // rot2 = Eigen::AngleAxisd(theta, -tilt_axis).toRotationMatrix();
        rot1 = DyrosMath::angleaxis2rot(tilt_axis, theta);
        rot2 = DyrosMath::angleaxis2rot(-tilt_axis, theta);
        

        o1 = rot1*Og_a;
        o2 = rot2*Og_a;

        // By geometric constrains, {G} is located along the negative direction of z_a axis;

        if(abs(o1(2)) <= abs(o2(2)))
        {   
            tilt_axis = -tilt_axis;
            std::cout<<"o1 is farther than o2"<<std::endl;
            std::cout<<"tilt_zix: "<<tilt_axis.transpose()<<std::endl;

        } 
        else 
        {   
            tilt_axis = tilt_axis;
            std::cout<<"o2 is farther than o1"<<std::endl;
            std::cout<<"tilt_zix: "<<tilt_axis.transpose()<<std::endl;
        }
            


        std::cout<<"R_ga: \n"<<R_ga<<std::endl;
        std::cout<<"Oa_g: "<<Oa_g.transpose()<<std::endl;
        std::cout<<"Og_a: "<<Og_a.transpose()<<std::endl;
        std::cout<<"N_g: " << N_g.transpose()<<std::endl;
        std::cout<<"o_1: "<< o1.transpose()<<std::endl;
        std::cout<<"o_2: "<< o2.transpose()<<std::endl;
        std::cout<<"o_g: "<<(R_ga*o1).transpose()<<std::endl;
        
        return tilt_axis;
    }

    static bool setTilt(const Matrix4d T_ga, const Vector3d assembly_dir, const double threshold)
    {
        Vector3d proj_vector;
        Vector3d p;
        double dis;
        bool set_tilt;

        dis = 0.0;
        p = T_ga.block<3,1>(0,3);

        for(int i = 0; i < 3; i++)
        {
            proj_vector(i) = 1.0 - round(assembly_dir(i));
            double temp = proj_vector(i)*p(i);
            dis += pow(temp,2);
        }

        dis = sqrt(dis);

        if(dis >= threshold) set_tilt = true;
        else set_tilt = false;

        return set_tilt;
    }

    static Vector6d tiltMotion(const Vector3d pos_init, const Matrix3d ori_init, const Vector3d position, const Matrix3d rotation,
                                const Vector6d xd, const Matrix4d T_ga, const Vector3d tilt_axis, const double tilt_angle,
                                const double t, const double t_0, const double duration)
    {
        Vector4d pos_target;
        Vector3d pos_desired, delphi_delta;
        Vector3d N_g;
        Matrix3d R_ga, R_aa, rot_target, rot_desired;
        
        Matrix3d K_p, K_v; 
        Vector3d f_star, m_star;

        Matrix4d T_ag, T_aa, T_wg;
        Vector4d Og_a, Ot_a, Ot_g;

        Vector6d f_star_zero;

        Og_a.Ones();
        Ot_a.Ones();
        Ot_g.Ones();

        K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
        K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;
        
        R_ga = T_ga.block<3,3>(0,0);
        T_ag = PegInHole2::setTransformationInverse(T_ga);        
        Og_a = T_ag.block<4,1>(0,3);
        R_aa = DyrosMath::angleaxis2rot(tilt_axis, tilt_angle);
        T_aa = PegInHole2::setTransformation(Eigen::Vector3d::Zero(), R_aa);
        T_wg = PegInHole2::setTransformation(pos_init, ori_init);

        N_g = R_ga*tilt_axis;
        // AngleAxisd(tilt_angle, tilt_axis).toRotationMatrix();        
        
        Ot_a = T_aa*Og_a;
        Ot_g = T_ga*Ot_a;

        pos_target = T_wg*T_ga*Ot_a;
        // rot_target = (T_wg*T_ga*T_aa).block<3,3>(0,0);
        rot_target = T_wg.block<3,3>(0,0)*DyrosMath::angleaxis2rot(N_g, tilt_angle);

        for(int i = 0; i < 3; i++) pos_desired(i) = DyrosMath::cubic(t, t_0, t_0 + duration, pos_init(i), pos_target(i), 0.0, 0.0);
        rot_desired = DyrosMath::rotationCubic(t, t_0, t_0 + duration, ori_init, rot_target);
        delphi_delta = -0.5 * DyrosMath::getPhi(rotation, rot_desired);

        f_star = K_p * (pos_desired.head<3>() - position) + K_v * (- xd.head<3>());
        m_star = (1.0) * 250.0 * delphi_delta + 2.0*(-xd.tail<3>());
        
        std::cout<<"-------------------------------"<<std::endl;
        std::cout<<"tilt axis: "<<tilt_axis.transpose()<<std::endl;
        std::cout<<"R_ga: \n"<<R_ga<<std::endl;
        std::cout<<"R_aa: \n"<<R_aa<<std::endl;
        std::cout<<"Og_a: "<<Og_a.transpose()<<std::endl;
        std::cout<<"Ot_a: "<<Ot_a.transpose()<<std::endl;        
        std::cout<<"Ot_g: "<<Ot_g.transpose()<<std::endl;
        std::cout<<"start:  "<<pos_init.transpose()<<std::endl;
        std::cout<<"target: "<<pos_target.transpose()<<std::endl;
        // std::cout<<"R_wg: \n"<<T_wg.block<3,3>(0,0)<<std::endl;
        // std::cout<<"R_wa: \n"<<(T_wg*T_ga*T_aa).block<3,3>(0,0)<<std::endl;
        // std::cout<<"Euler R_wg: "<<DyrosMath::rot2Euler(T_wg.block<3,3>(0,0)).transpose()*RAD2DEG<<std::endl;
        // std::cout<<"Euler R_wa_init: "<<DyrosMath::rot2Euler((T_wg*T_ga).block<3,3>(0,0)).transpose()*RAD2DEG<<std::endl;
        // std::cout<<"Euler R_wa_final: "<<DyrosMath::rot2Euler((T_wg*T_ga*T_aa).block<3,3>(0,0)).transpose()*RAD2DEG<<std::endl;

        f_star_zero << f_star, m_star;
        return f_star_zero;
    }



    // static bool::judgeHeavyMass(const double t, const double )


    static Vector3d straightMoveEE(const Vector3d origin,
        const Matrix3d ori_init,
        const Vector3d current_position,
        const Eigen::Matrix<double, 6, 1> xd,        
        const Vector3d dir,
        const double speed,
        const double current_time,
        const double init_time)
    {
        // double desent_speed = -0.02; //-0.005; // 5cm/s

        Vector3d desired_position, desired_velocity;
        Vector3d f_star;
        Matrix3d K_p, K_v; 
        Vector3d x_ee, v_ee;
      
        K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
        K_v << 100, 0, 0, 0, 100, 0, 0, 0, 100;
        
        x_ee.setZero();
        v_ee.setZero();

        x_ee = speed*(current_time - init_time)*dir;       
        v_ee = speed*dir;
        
        // offset_init = ori_init*DyrosMath::rotateWithZ(M_PI_4); //Only For the simulation!!!

        // desired_position = origin + offset_init*x_ee;
        // desired_velocity = offset_init*v_ee;
                

        desired_position = origin + ori_init*x_ee;
        desired_velocity = ori_init*v_ee;

        f_star = K_p * (desired_position - current_position) + K_v * (desired_velocity - xd.head<3>());  
        
        return f_star;
    }
    
    static Vector3d oneDofMoveEE(const Vector3d origin,
        const Matrix3d ori_init,
        const Vector3d current_position,
        const double target_distance,
        const Eigen::Matrix<double, 6, 1> xd,
        const double current_time,
        const double init_time,
        const double duration,
        const Vector3d dir)
    {
        Vector3d desired_position;
        Vector3d f_star;
        Matrix3d K_p, K_v; 
        Vector3d target_position;
        
        K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
        K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

        target_position = target_distance*dir;

        for(int i = 0; i < 3; i ++)
        {
            desired_position(i) = DyrosMath::cubic(current_time, init_time, init_time + duration, 0.0, target_position(i), 0, 0);
        }

        desired_position = origin + ori_init*desired_position;

        f_star = K_p * (desired_position - current_position) + K_v * (- xd.head<3>());  
       
        return f_star;    
        // return desired_position;
    }

    static Vector3d twoDofMove(const Vector3d origin,
        const Vector3d current_position,
        const Vector3d target_position,
        const Eigen::Matrix<double, 6, 1> xd,
        const double current_time,
        const double init_time,
        const double duration,
        const double desired_speed,
        const int direction) // 0 -> x, 1 -> y, 2 -> z //NOT DESIRED DIRECTION!!
    {
        Vector3d speed; // x, y, z
        Vector3d desired_position;
        Vector3d desired_velocity;
        Vector3d f_star;
        Matrix3d K_p; 
        Matrix3d K_v;
        
        K_p << 5500, 0, 0, 0, 5500, 0, 0, 0, 5500;
        K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

        for(size_t i = 0; i < 3; i++)
        {
            if(origin(i) < target_position(i)) speed(i) = desired_speed;
            else speed(i) = -desired_speed;
        }

        if(direction == 0) //NOT WANT TO MOVE ALONG X - AXIS
        {
            desired_position(0) = target_position(0);
            desired_position(1) = DyrosMath::cubic(current_time, init_time, init_time + duration, origin(1), target_position(1), 0, 0);
            desired_position(2) = DyrosMath::cubic(current_time, init_time, init_time + duration, origin(2), target_position(2), 0, 0);
            desired_velocity << 0, speed(1), speed(2);
        }
        if(direction == 1)
        {
            desired_position(0) = DyrosMath::cubic(current_time, init_time, init_time + duration,origin(0),target_position(0),0,0);
            desired_position(1) = target_position(1);
            desired_position(2) = DyrosMath::cubic(current_time, init_time, init_time + duration,origin(2),target_position(2),0,0);
            desired_velocity << speed(0), 0, speed(2);
        }
        if(direction == 2) //NOT WANT TO MOVE ALONG Z - AXIS
        {
            desired_position(0) = DyrosMath::cubic(current_time, init_time, init_time + duration, origin(0), target_position(0), 0, 0);
            desired_position(1) = DyrosMath::cubic(current_time, init_time, init_time + duration, origin(1), target_position(1), 0, 0);
            desired_position(2) = target_position(2);
            desired_velocity << speed(0), speed(1), 0;
        }
        
        f_star = K_p * (desired_position - current_position) + K_v * ( desired_velocity- xd.head<3>());  
        // std::cout<<"-------------------------------------------"<<std::endl;
        // std::cout<<"target_position : "<<target_position.transpose()<<std::endl;
        // std::cout<<"current_position : "<<current_position.transpose()<<std::endl;
        // std::cout<<"desired speed : "<<desired_velocity.transpose()<<std::endl;
        // std::cout<<"current speed : "<<xd.head<3>().transpose()<<std::endl;
        // std::cout<<"f_star : "<<f_star.transpose()<<std::endl;
        return desired_velocity;    
    }

    

    static Eigen::Matrix<double, 6, 1> keepCurrentState(const Vector3d initial_position,
        const Matrix3d initial_rotation,
        const Vector3d position,
        const Matrix3d rotation,
        const Eigen::Matrix<double, 6, 1> xd, 
        const double kp = 5000, const double kv = 100)
    {
        Vector3d desired_position;
        Vector3d desired_linear_velocity;
        Vector3d delphi_delta;
        Vector3d f_star;
        Vector3d m_star;
        Eigen::Vector6d f_star_zero;
        Matrix3d kp_m; 
        Matrix3d kv_m;

        kp_m = Matrix3d::Identity() * kp;
        kv_m = Matrix3d::Identity() * kv;

        desired_position = initial_position;
        desired_linear_velocity.setZero();

        delphi_delta = -0.5 * DyrosMath::getPhi(rotation, initial_rotation);

        f_star = kp_m * (desired_position - position) + kv_m * ( desired_linear_velocity- xd.head<3>());  
        m_star = (1.0) * 300.0* delphi_delta+ 5.0*(-xd.tail<3>());

        f_star_zero.head<3>() = f_star;
        f_star_zero.tail<3>() = m_star;

        return f_star_zero;    
    }    

    static Vector3d  generateSpiral(const Vector3d origin, 
        const Vector3d current_position,
        const Eigen::Matrix<double, 3, 1> xd,
        const double pitch,
        const double lin_vel,
        const int dir, //the direction where a peg is inserted
        const double current_time,
        const double init_time,
        const double spiral_duration)
    {
        // double pitch = 0.0010; 
        // double lin_vel = 0.005; 
        Eigen::Vector2d start_point;
        Eigen::Vector2d traj;
        Vector3d desired_position;
        Vector3d desired_linear_velocity;
        Vector3d f_star;
        Matrix3d K_p;
        Matrix3d K_v;

        Vector3d real_origin;
        Vector3d real_current_position;

        K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
        K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;

        if(dir == 0) start_point << origin(1), origin(2);
        if(dir == 1) start_point << origin(0), origin(2);
        if(dir == 2) start_point << origin(0), origin(1);
        
        traj = DyrosMath::spiral(current_time, init_time, init_time + spiral_duration, start_point, lin_vel, pitch);

        if(dir == 0) desired_position << origin(dir), traj(0), traj(1);
        if(dir == 1) desired_position << traj(0), origin(dir), traj(1);
        if(dir == 2) desired_position << traj(0), traj(1), origin(dir);
        
        desired_linear_velocity.setZero();
    
        f_star = K_p * (desired_position - current_position) + K_v * (desired_linear_velocity- xd.head<3>());  
            
        return f_star;
    }
    
    static Vector3d  generateSpiralEE(const Vector3d origin, 
        const Matrix3d ori_init,
        const Vector3d current_position,
        const Vector6d xd,
        const double pitch,
        const double lin_vel,
        const Matrix4d T_ea, //the direction where a peg is inserted, wrt {E} .i.e., T_ga
        const double current_time,
        const double init_time,
        const double spiral_duration)
    {
        Vector2d start_point, traj;
        Vector4d p_a, p_w; // wrt frame {A}
        Vector3d desired_position; // wrt frame {W}
        Vector3d f_star;
        Matrix3d K_p, K_v;
        Matrix4d T_wa, T_we;

        K_p << 5000, 0, 0, 0, 5000, 0, 0, 0, 5000;
        K_v << 200, 0, 0, 0, 200, 0, 0, 0, 200;                        
        start_point.setZero();

        T_we = PegInHole2::setTransformation(origin, ori_init);
        T_wa = T_we*T_ea;

        traj = DyrosMath::spiral(current_time, init_time, init_time + spiral_duration, start_point, lin_vel, pitch);
        
        p_a << traj(0), traj(1), 0.0, 1.0;        
        p_w = T_wa*p_a;

        desired_position = p_w.head<3>();
        
    
        f_star = K_p * (desired_position - current_position) + K_v * (- xd.head<3>());  
            
        return f_star;
    }

    static Vector3d generateTwistEE(const Matrix3d &ori_init, const Matrix3d &rotation, const Vector6d &xd,
        const double theta_max,
        const double theta_dot,
        const Vector3d &assembly_dir,
        const Matrix4d &T_EA,
        const double t,
        const double t_0)
    {
        Matrix3d target_rotation, current_rotation, rot_a;
        Vector3d delphi_delta;
        Vector3d m_ee, m_a;
        double theta, omega;                      
        double t_e;
                        
        t_e = t - t_0;
        omega = 2*M_PI/(2*(theta_max/theta_dot));
        
        theta = theta_max*sin(omega*t_e);

        rot_a = DyrosMath::rotateWithZ(theta);
        target_rotation = T_EA.block<3,3>(0,0)*rot_a; //wrt {E}
        current_rotation = ori_init.transpose()*rotation; //wrt {E}
                
        delphi_delta = -0.5 * DyrosMath::getPhi(current_rotation, target_rotation);        

        m_ee = (1.0) * 200.0* delphi_delta;

        return m_ee;
    }

    static Eigen::Matrix<double, 3, 1>  generateSpiralWithRotation(const Matrix3d initial_rotation_M, 
        const Matrix3d rotation_M,
        const Eigen::Matrix<double, 3, 1> current_angular_velocity,
        const double current_time,
        const double init_time,
        const double duration,
        const double direction,  //0 = the first motion, 1 = CCW, 2 == CW
        const double axis) // 0 = x-axis, 1 = y-axis, 2 = z-axis
    {
        double target_angle = 3.0*M_PI/180;
        double ori_change_theta;
        
        Matrix3d rotation_matrix;
        Matrix3d target_rotation_M;  
        Vector3d delphi_delta;
        Vector3d m_star;
                
        if(direction == 0.0)
        {
            ori_change_theta = DyrosMath::cubic(current_time, init_time, init_time + duration/2, 0, target_angle, 0, 0);    
        }
        
        if(direction == 1.0)
        {
            ori_change_theta = DyrosMath::cubic(current_time, init_time, init_time + duration, target_angle, -target_angle, 0, 0);    
        }          

        if(direction == 2.0)
        {
            ori_change_theta = DyrosMath::cubic(current_time, init_time, init_time + duration, -target_angle, target_angle, 0, 0);
        }            
        
        if(axis == 0) rotation_matrix << 1, 0, 0, 0, cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta);
        if(axis == 1) rotation_matrix << cos(ori_change_theta), 0, sin(ori_change_theta), 0, 1, 0, -sin(ori_change_theta), 0, cos(ori_change_theta);
        if(axis == 2) rotation_matrix << cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta), 0, 0, 0, 1;
        
        target_rotation_M = rotation_matrix * initial_rotation_M;

        delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);
        
        m_star = (1.0) * 200.0* delphi_delta+ 5.0*(-current_angular_velocity);
                
        return m_star;
    }

    static Eigen::Matrix<double, 3, 1>  generateSpiralWithRotationGainEe(const Matrix3d initial_rotation_M, 
        const Matrix3d rotation_M,
        const Eigen::Matrix<double, 3, 1> current_angular_velocity,
        const double current_time,
        const double init_time,
        const double duration,
        const double direction,  //0 = the first motion, 1 = CCW, 2 == CW
        const double axis,  // 0 = x-axis, 1 = y-axis, 2 = z-axis
        const double kp,
        const double kv)
    {
        double target_angle = 3.0*M_PI/180;
        double ori_change_theta;
        
        Matrix3d rotation_matrix;
        Matrix3d target_rotation_M;  
        Vector3d delphi_delta;
        Vector3d m_star;
        Matrix3d K_p; 
        Matrix3d K_v;

        K_p << kp, 0, 0, 0, kp, 0, 0, 0, kp;
        K_v << kv, 0, 0, 0, kv, 0, 0, 0, kv;
                
        if(direction == 0.0)
        {
            ori_change_theta = DyrosMath::cubic(current_time, init_time, init_time + duration/2, 0, target_angle, 0, 0);    
        }
        
        if(direction == 1.0)
        {
            ori_change_theta = DyrosMath::cubic(current_time, init_time, init_time + duration, target_angle, -target_angle, 0, 0);    
        }          

        if(direction == 2.0)
        {
            ori_change_theta = DyrosMath::cubic(current_time, init_time, init_time + duration, -target_angle, target_angle, 0, 0);
        }            
        
        if(axis == 0) rotation_matrix << 1, 0, 0, 0, cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta);
        if(axis == 1) rotation_matrix << cos(ori_change_theta), 0, sin(ori_change_theta), 0, 1, 0, -sin(ori_change_theta), 0, cos(ori_change_theta);
        if(axis == 2) rotation_matrix << cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta), 0, 0, 0, 1;
        
        target_rotation_M = initial_rotation_M * rotation_matrix;

        delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);
        
        m_star = (1.0) * K_p * delphi_delta+ K_v * (-current_angular_velocity);
                
        return m_star;
    }

    static Eigen::Matrix<double, 3, 1>  generateSpiralWithRotationEe(const Matrix3d initial_rotation_M, 
        const Matrix3d rotation_M,
        const Eigen::Matrix<double, 3, 1> current_angular_velocity,
        const double current_time,
        const double init_time,
        const double duration,
        const double direction,  //0 = the first motion, 1 = CCW, 2 == CW
        const double axis,
        const int rand) // 0 = x-axis, 1 = y-axis, 2 = z-axis
    {
        double target_angle = 1.5*M_PI/180;
        double ori_change_theta;
        
        Matrix3d rotation_matrix;
        Matrix3d target_rotation_M;  
        Vector3d delphi_delta;
        Vector3d m_star;

        
        if(direction == 0.0)
        {
            ori_change_theta = DyrosMath::cubic(current_time, init_time, init_time + duration/2, 0, target_angle, 0, 0);    
        }
        
        if(direction == 1.0)
        {
            ori_change_theta = DyrosMath::cubic(current_time, init_time, init_time + duration, target_angle, -target_angle, 0, 0);    
        }          

        if(direction == 2.0)
        {
            ori_change_theta = DyrosMath::cubic(current_time, init_time, init_time + duration, -target_angle, target_angle, 0, 0);
        }

        if(direction == 3.0)
        {
            ori_change_theta = DyrosMath::cubic(current_time, init_time, init_time + duration/2, 0, -target_angle, 0, 0);    
        }            
        
        if(axis == 0) rotation_matrix << 1, 0, 0, 0, cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta);
        if(axis == 1) rotation_matrix << cos(ori_change_theta), 0, sin(ori_change_theta), 0, 1, 0, -sin(ori_change_theta), 0, cos(ori_change_theta);
        if(axis == 2) rotation_matrix << cos(ori_change_theta), -sin(ori_change_theta), 0, sin(ori_change_theta), cos(ori_change_theta), 0, 0, 0, 1;
        
        target_rotation_M = initial_rotation_M * rotation_matrix;

        delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);
        
        m_star = (1.0) * 200.0* delphi_delta+ 5.0*(-current_angular_velocity);
                
        return m_star;
    }

    static Vector3d keepOrientationPerpenticular(const Matrix3d initial_rotation_M,
		const Matrix3d rotation_M,
		const Eigen::Matrix<double, 6, 1> xd,
		const double duration,
		const double current_time,
		const double init_time)
	{
		Matrix3d target_rotation_M;
		Vector3d delphi_delta;
		Vector3d m_star;
		Eigen::Vector5d angle_set_45;
		Eigen::Vector5d angle_set_error;
		Vector3d euler_angle;

		double val;
		double e;

		double roll, alpha;
		double pitch, beta;
		double yaw, gamma;

		double min;
		int index;

		euler_angle = DyrosMath::rot2Euler(initial_rotation_M);
		roll = euler_angle(0);
		pitch = euler_angle(1);
		yaw = euler_angle(2);

		val = initial_rotation_M(2, 2);
		e = 1.0 - fabs(val);
        
		angle_set_45 << -135.0, -45.0, 45.0, 135.0; //In the simulatrion
        // angle_set_45 << -180.0, -90.0, 0.0, 90.0, 180.0; // In the real robot application
		angle_set_45 = angle_set_45 * DEG2RAD;

		if (val > 0 && e <= 0.01) //upward
		{
			roll = 0;
			pitch = 0;

			for (size_t i = 0; i < 5; i++)
			{
				angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(2));
			}

			for (size_t i = 0; i < 5; i++)
			{
				if (angle_set_error(i) == angle_set_error.minCoeff()) index = i;
			}


			yaw = angle_set_45(index);

		}
		else if (val < 0 && e <= 0.01) //downward
		{
			if (roll > 0) roll = 180 * DEG2RAD;
			else roll = -180 * DEG2RAD;

			pitch = 0;

			for (size_t i = 0; i < 5; i++)
			{
				angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(2));
			}
			for (size_t i = 0; i < 5; i++)
			{
				if (angle_set_error(i) == angle_set_error.minCoeff()) index = i;
			}

			yaw = angle_set_45(index);
		}

		else //on xy plane
		{
			roll = euler_angle(0);
			yaw = euler_angle(2);

			for (size_t i = 0; i < 5; i++)
			{
				angle_set_error(i) = fabs(angle_set_45(i) - euler_angle(1));
			}

			for (size_t i = 0; i < 5; i++)
			{
				if (angle_set_error(i) == angle_set_error.minCoeff()) index = i;
			}

			pitch = angle_set_45(index);
		}

		alpha = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(0), roll, 0, 0);
		beta = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(1), pitch, 0, 0);
		gamma = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(2), yaw, 0, 0);

		target_rotation_M = DyrosMath::rotateWithZ(gamma) * DyrosMath::rotateWithY(beta) * DyrosMath::rotateWithX(alpha);

		delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);

		m_star = (1.0) * 250.0 * delphi_delta + 1.0 * (-xd.tail<3>());

        Vector3d desired_euler;

        desired_euler << alpha, beta, gamma;

		return m_star;
        // return desired_euler;
	}    
    
    static Vector3d keepOrientationPerpenticularOnlyXY(const Matrix3d initial_rotation_M,
        const Matrix3d rotation_M,
        const Eigen::Matrix<double, 6, 1> xd,
        const double duration,
        const double current_time,
        const double init_time)
    {
        Matrix3d target_rotation_M;
        Vector3d delphi_delta;
        Vector3d m_star;
        Vector3d euler_angle;

        double val;
        double e;

        double roll, alpha;
        double pitch, beta;
        double yaw, gamma;

        euler_angle = DyrosMath::rot2Euler(initial_rotation_M);
        roll = euler_angle(0);
        pitch = euler_angle(1);
        yaw = euler_angle(2);

        val = initial_rotation_M(2, 2);
        e = 1.0 - fabs(val);

        if (val > 0) //upward
        {
            roll = 0;
            pitch = 0;
        }
        
        else //downward
        {
            if (roll > 0) roll = 180 * DEG2RAD;
            else roll = -180 * DEG2RAD;
            pitch = 0;
        }

        alpha = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(0), roll, 0, 0);
        beta = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(1), pitch, 0, 0);
        gamma = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(2), yaw, 0, 0);

        target_rotation_M = DyrosMath::rotateWithZ(gamma) * DyrosMath::rotateWithY(beta) * DyrosMath::rotateWithX(alpha);

        delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);

        m_star = (1.0) * 250.0 * delphi_delta + 5.0 * (-xd.tail<3>());

        return m_star;
    }

    static Vector3d rotateOrientationPerpenticular(const Matrix3d initial_rotation_M,
		const Matrix3d rotation_M,
		const Eigen::Matrix<double, 6, 1> xd,
		const double duration,
		const double current_time,
		const double init_time)
	{
        Matrix3d target_rotation_M;
        Vector3d delphi_delta;
        Vector3d m_star;
        Vector3d euler_angle;

        double val;
        double e;

        double roll, alpha;
        double pitch, beta;
        double yaw, gamma;

        euler_angle = DyrosMath::rot2Euler(initial_rotation_M);
        roll = euler_angle(0);
        pitch = euler_angle(1);
        yaw = euler_angle(2);

        val = initial_rotation_M(2, 2);
        e = 1.0 - fabs(val);

        if (val > 0) //upward
        {
            roll = 0;
            pitch = 0;
        }
        
        else //downward
        {
            if (roll > 0) roll = 180 * DEG2RAD;
            else roll = -180 * DEG2RAD;
            pitch = 0;
        }

        alpha = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(0), roll, 0, 0);
        beta = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(1), pitch + (3*M_PI/180), 0, 0);
        gamma = DyrosMath::cubic(current_time, init_time, init_time + duration, euler_angle(2), yaw, 0, 0);

        target_rotation_M = DyrosMath::rotateWithZ(gamma) * DyrosMath::rotateWithY(beta) * DyrosMath::rotateWithX(alpha);

        delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);

        m_star = (1.0) * 250.0 * delphi_delta + 5.0 * (-xd.tail<3>());

        return m_star;
	}    

    static Vector3d rotateWithGlobalAxis(const Matrix3d initial_rotation_M,
        const Matrix3d rotation_M,
        const Eigen::Matrix<double, 6, 1> xd,
        const double goal,
        const double init_time,
        const double current_time,
        const double end_time,
        const int dir) // 0 = x-axis, 1 = y-axis, 2 = z-axis
    {
        Matrix3d rot;
        Matrix3d target_rotation_M;
        Vector3d delphi_delta;
        Vector3d m_star;
        

        double theta;
        double run_time;
        double duration = (end_time - init_time)/4.0; //0.5s
        
        run_time = current_time - init_time;

        
        if(run_time < duration)
        {
            theta = DyrosMath::cubic(current_time, init_time, init_time + duration, 0, goal, 0, 0);        
        } 

        else if(duration <= run_time && run_time < 3*duration)
        {
            theta = DyrosMath::cubic(current_time, init_time + duration, init_time + 3*duration, goal, -goal, 0, 0);            
        }

        else// if(3*duration/2 <= run_time && run_time < 2*duration)
        {
            theta = DyrosMath::cubic(current_time, init_time + 3*duration, init_time + 4*duration, -goal, 0, 0, 0);
        }
                
        
        if(dir == 0)    rot << 1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta);
        if(dir == 1)    rot << cos(theta), 0, sin(theta), 0, 1, 0, -sin(theta), 0, cos(theta);
        if(dir == 2)    rot << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
        
        
        target_rotation_M = rot * initial_rotation_M;

        delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);

        m_star = (1.0) * 200.0 * delphi_delta + 5.0*(-xd.tail<3>());

                
        // std::cout<<"----------------------"<<std::endl;
        // std::cout<<"time: "<<run_time<<std::endl;
        // std::cout<<"duration : "<<duration<<std::endl;
        // std::cout<<"theta: "<<theta*180/M_PI<<std::endl;
        return m_star;
    }

    static Vector3d rotateWithEeAxis(const Matrix3d initial_rotation_M,
        const Matrix3d rotation_M,
        const Eigen::Matrix<double, 6, 1> xd,
        const double goal,
        const double t,
        const double t_0,
        const double duration,
        const Vector3d dir) //axis_vector wrt EE frame
    {
        Matrix3d rot;
        Matrix3d target_rotation_M;
        Vector3d delphi_delta;
        Vector3d m_star;
        

        double theta;

        theta = DyrosMath::cubic(t, t_0, t_0 + duration, 0, goal, 0, 0);        
        
        
        rot = DyrosMath::angleaxis2rot(dir, theta);               
                                
        target_rotation_M = initial_rotation_M * rot;

        delphi_delta = -0.5 * DyrosMath::getPhi(rotation_M, target_rotation_M);

        m_star = (1.0) * 200.0 * delphi_delta + 5.0*(-xd.tail<3>());

        return m_star;
    }

    static double calculateFriction(const int dir, const Vector3d f_measured, double friction)
    {
        if(dir == 0) friction = sqrt(f_measured(1)*f_measured(1) +f_measured(2)*f_measured(2));
        if(dir == 1) friction = sqrt(f_measured(0)*f_measured(0) +f_measured(2)*f_measured(2));
        if(dir == 2) friction = sqrt(f_measured(0)*f_measured(0) +f_measured(1)*f_measured(1));

        return friction;
    }

   
    static Vector3d vectorNormalization(const Vector3d v)
    {
        Vector3d n;

        n = v/sqrt(pow(v(0),2) + pow(v(1),2) + pow(v(2),2));

        return n;
    }
    static Vector3d computeNomalVector(const Vector3d p1,
        const Vector3d p2,
        const Vector3d p3)
    {
        Vector3d r1;
        Vector3d r2;
        Vector3d r3;
        Vector3d result;

        r1 = p1 - p2;
        r2 = p1 - p3;

        r3 = r1.cross(r2);
        r3 = r3/sqrt(pow(r3(0),2) + pow(r3(1),2) + pow(r3(2),2));

        result = r3;
        return result;
    }

    static Matrix3d generateWiggleMotion(const Matrix3d init_rot, const Matrix3d cur_rot, const Vector3d xdot_w,
                                                    const double angle, const int dir, const double t, const double t_0, const double duration)
    {
        Matrix3d wiggle_rot;
        Matrix3d target_rot;
        Vector3d m_star;
        Vector3d delphi_delta;

        Matrix3d kp = Matrix3d::Identity();
        Matrix3d kv = Matrix3d::Identity();

        kp = 100*kp;
        kp(1,1) = 500.0;
        kp(2,2) = 500.0;

        kv = 2.0*kv;
        // kv(1,1) = 7.5;

        double t_f;
        double t_e;
        double w;
        double theta;

        t_f = t_0 + duration;
        t_e = t - t_0;
        w = 2*M_PI/duration;
        
        theta = angle*sin(w*t_e);

        switch(dir)
        {
            case 0:
                wiggle_rot = init_rot*DyrosMath::rotateWithX(theta); 
                break;
            case 1:
                wiggle_rot = init_rot*DyrosMath::rotateWithY(theta);
                break;
            case 2:
                wiggle_rot = init_rot*DyrosMath::rotateWithZ(theta);
        }

        delphi_delta = -0.5 * DyrosMath::getPhi(cur_rot, wiggle_rot);
    
        m_star = (1.0) * kp * delphi_delta - kv*xdot_w;
        
        // std::cout<<output<<std::endl;
        // return m_star;
        return wiggle_rot;
    }
   
   
};
