#ifndef CRITERIA_H
#define CRITERIA_H

#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

#include "math_type_define.h"
#include "fuzzycontrol.h"
#include "crispcontrol.h"

using namespace DyrosMath;
using namespace FuzzyLogic;
using namespace CrispLogic;

namespace Criteria
{
    static bool checkContact(const double current_force, const double threshold)
    {
        double contact_force = threshold; //-6.0
        double input_force = current_force;
        bool result;
        
        if(abs(input_force) >= abs(contact_force))  result = true;
        else result = false;

        return result;
    }

    static bool getCount(const int cnt, const int threshold)
    {
        bool result;
        
        if(cnt > threshold) result = true;
        else result = false;

        return result;
    }

    static bool checkDisplacement(const double origin,
        const double current_position,        
        const double base_position, //0.270
        const double component_info) // the distance from end-effecto to the edge of the component
    {
        double safty = 0.0005;
        double current_dis;
        double target_dis;
        bool result;

        current_dis = origin - current_position;
        target_dis = origin - component_info - base_position ;

        if(current_dis <= target_dis) result = true;
        else result = false;
               
        return result;
    }
    
    static bool detectHole(const double origin,
        const double current_position,
        const double friction,
        const double depth_threshold,
        const double force_threshold)
    {
        double dz;
        bool result; // true = success, false = fail

        dz = origin - current_position;

        // Keep considering conditions, which is || or &&
        if(dz >= depth_threshold && friction >= force_threshold)
        {
            //std::cout<<"Z_l + F_L"<<std::endl;
            result = true;  //z_l + f_l
        } 
        else if(dz >= depth_threshold && friction < force_threshold)
        {
            //std::cout<<"Z_l + F_s"<<std::endl;
            result = true; //z_l + f_s
        } 
        else if(dz < depth_threshold && friction >= force_threshold)
        {
            //std::cout<<"Z_s + F_l"<<std::endl;
            //std::cout<<dz<<"    "<<friction<<std::endl;
            result = true; //z_s + f_l
        } 
        else result = false;  //z_s + f_s        
        return result;
    }

    static double judgeInsertion(const double origin,
        const double current_position,
        const double current_velocity,
        const double friction,
        const double target_depth)
    {
        double velocity_threshold = 0.01; // fix it later
        double force_threshold = 15.0;
        double depth_threshold;
        double dz;
        double result; //1.0 = success, 0.0 = ?, -1.0 = fail(dead or fake)

        depth_threshold = target_depth*0.8; // 
        dz = origin - current_position;
        result = 0.0;

        if(abs(current_velocity) <= velocity_threshold)
        {
            if(dz < depth_threshold && friction <= force_threshold)
            {
                std::cout<<"keep inserting"<<std::endl;
                result = 0.0; //not to consider
            } 
            else if(dz < depth_threshold && friction > force_threshold)
            {
                std::cout<<"fake"<<std::endl;
                result = -1.0; // fake
            } 
            else if((depth_threshold <= dz && dz <= target_depth) && friction <= force_threshold)
            {
                std::cout<<"dead"<<std::endl;
                result = -1.0; // dead
            } 
            else if((depth_threshold <= dz && dz <= target_depth) && friction > force_threshold)
            {
              std::cout<<"success"<<std::endl;
              result = 1.0; // success  
            } 
            else
            {
                std::cout<<"dead"<<std::endl;
                result = -1.0; // dead
            } 
        }

        else
        {
            if(dz >= target_depth) result = -1.0;               
        }        
    } 

    static bool timeOut(const double current_time, const double start_time, const double duration)
    {
        double running_time;
        bool is_timeout;

        running_time = current_time - start_time;

        if(running_time >= duration) is_timeout = true;
        else is_timeout = false;
        
        return is_timeout;
    }
    
    static bool checkForceLimit(const double f,
        const double threshold)        //be absolute value!!
    {   
        bool is_done;
        
        
        if(f >= threshold) is_done = true;
        else is_done = false;
                
        return is_done;
    }

    static bool checkForceDot(const std::vector<double> vec, const double threshold)
    {
        bool is_done;
        double f1,f2;
        double del_f;
        std::vector<double> force;

        force = vec;

        if(vec.size() < 2) del_f = 0.0;
        else
        {
            f1 = force.back();
            force.pop_back();
            f2 = force.back();

            del_f = abs(f2 - f1);
        }          

        if(del_f >= threshold) is_done = true;
        else is_done = false;

        return is_done;
    }

    static bool checkMomentLimit(const std::vector<double> m1,
        const std::vector<double> m2,
        const std::vector<double> m3,
        const int swing_dir,
        const double threshold)        
    {   
        bool is_done;
        int size;
        double sum;
        double avg;
        
        std::vector<double> m;

        if(swing_dir == 1) m = m1;
        if(swing_dir == 2) m = m2;
        if(swing_dir == 3) m = m3;

        size = int(m.size());

        for(int i = 0; i < size; i ++)
        {
            sum += fabs(double(m[i]));
        }
        
        avg = sum / size;

        if(avg >= threshold) is_done = true;
        else is_done = false;

        // std::cout<<"size : "<<size<<std::endl;
        // std::cout<<"sum : "<<sum<<std::endl;
        std::cout<<"moment avg : "<<avg<<std::endl;
        // std::cout<<"is_done : "<<is_done<<std::endl;
        
        return is_done;
    }

    // static bool checkSideChairDone(const std::vector<double> v1,
    //     const std::vector<double> v2,
    //     const std::vector<double> v3)
    // {
    //     bool is_done;

    //     auto x = leastSquareLinear(v1,1000);
    //     auto y = leastSquareLinear(v2,1000);
    //     auto z = leastSquareLinear(v3,1000);

    //     std::cout<<x.transpose()<<std::endl;
    //     std::cout<<y.transpose()<<std::endl;
    //     std::cout<<z.transpose()<<std::endl;

    //     return is_done;
    // }
    
    static double fuzzyLogic(const double origin, const double vel, const double dis, const double force)
    {
        Eigen::Vector2d v;
        Eigen::Matrix<double, 5, 1> z;
        Eigen::Vector3d f;

        double u;
        
        v = velocityInput(vel);        
        z = displacementInput(origin, dis);        
        f = forceInput(force);      
        
        u = fuzzyOutput(v,z,f);
        // std::cout<<"---------------------------------------"<<std::endl;
        return u;
    }

    static int crispLogic(const double origin, const double vel, const double dis, const double force)
    {
        int v;
        int z;
        int f;

        int u;
        
        v = velocityInputCrisp(vel);        
        z = displacementInputCrisp(origin, dis);        
        f = forceInputCrisp(force);      
        
        u = crispOutput(v,z,f);
        // std::cout<<"---------------------------------------"<<std::endl;
        // std::cout<<"vel: "<<vel<<std::endl;
        // stdLL
        return u;
    }
};

#endif // CRITERIA_H