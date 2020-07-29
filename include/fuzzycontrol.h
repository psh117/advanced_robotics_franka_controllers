#ifndef FUZZYCONTROL_H
#define FUZZYCONTROL_H

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

// #define MOVE 0.5
// #define DEAD 0.0
// #define HOLE 1.0
// #define FAKE -1.0

#define NONE        0
#define CS_ONE      1
#define CS_TWO      2
#define CS_THREE    3
#define CS_FOUR     4
#define CS_FIVE_ONE 5
#define CS_FIVE_TWO 6

namespace FuzzyLogic
{
    static Eigen::Vector2d velocityInput(const double z_vel)
    {
        //parameters fuzzy-logic-velocity function
        //--------------------
        double v_s = 0.006;
        double v_s_end = 0.007;
        //--------------------
        double v_b_start = 0.006;
        double v_b = 0.007;
        //--------------------
        double vel;
        

        Eigen::Vector2d result;
        result.setZero();

        vel = -z_vel;
        // vel = z_vel;

        // vel = 0.0069; //debug
        // vel = -0.0022;

        if(vel <= v_s)
        {
            result(0) = 1;
            result(1) = 0;
        }
            
        else if( v_s < vel && vel <= v_s_end)
        {
            result(0) = fabs(1/(v_s_end-v_s)*(vel - v_s_end));
            result(1) = 1/(v_b-v_b_start)*(vel - v_b_start);
        }
            
        else
        {
            result(0) = 0;
            result(1) = 1;
        }
        // std::cout<<"vel: "<<vel<<std::endl;
        // std::cout<<"vel_input:"<<result.transpose()<<std::endl;
        // std::cout<<"Fuzzifed velocity: "<<result.transpose()<<std::endl;
        return result;
    }

    // static Eigen::Matrix<double, 5, 1> displacementInput(const double origin, const double z_dis)
    // {
    //     //parameters fuzzy-logic-displacement function
    //     //--------------------
    //     double z_pvs = 0.0005;
    //     double z_pvs_end = 0.001;
    //     //--------------------
    //     double z_ps_start = 0.0005;
    //     double z_ps = 0.001;
    //     double z_ps_ = 0.0030;
    //     double z_ps_end = 0.0035;
    //     //--------------------
    //     double z_pm_start = 0.0030;
    //     double z_pm = 0.0035;
    //     double z_pm_ = 0.006;
    //     double z_pm_end = 0.0065;
    //     //--------------------
    //     double z_pb_start = 0.006;
    //     double z_pb = 0.0065;
    //     double z_pb_ = 0.012;
    //     double z_pb_end = 0.0125;
    //     //--------------------
    //     double z_pvb_start = 0.012;
    //     double z_pvb = 0.0125;
        
    //     double dis;
    //     // std::cout<<"spiral orign: "<<origin<<std::endl;
    //     Eigen::Matrix<double, 5, 1> result;
    //     result.setZero();

    //     dis = origin - z_dis;
        
    //     if( dis <= z_pvs)
    //     {
    //         result(0) = 1; // move
    //         result(1) = 0; // fake
    //         result(2) = 0; // shallow dead
    //         result(3) = 0; // hole
    //         result(4) = 0; // deep dead
    //     }
            
    //     else if( z_pvs < dis && dis <= z_pvs_end)
    //     {
    //         result(0) = fabs(1/(z_pvs_end - z_pvs)*(dis - z_pvs_end));
    //         result(1) = 1/(z_ps - z_ps_start)*(dis - z_ps_start);
    //         result(2) = 0;
    //         result(3) = 0;
    //         result(4) = 0;
    //     }
            
    //     else if( z_ps < dis && dis <= z_ps_)
    //     {
    //         result(0) = 0;
    //         result(1) = 1;
    //         result(2) = 0;
    //         result(3) = 0;
    //         result(4) = 0;
    //     }
            
    //     else if( z_ps_ < dis && dis <= z_ps_end)
    //     {
    //         result(0) = 0;
    //         result(1) = fabs(1/(z_ps_end - z_ps_)*(dis - z_ps_end));
    //         result(2) = 1/(z_pm - z_pm_start)*(dis - z_pm_start);
    //         result(3) = 0;
    //         result(4) = 0;
    //     }
            
    //     else if( z_pm < dis && dis <= z_pm_)
    //     {
    //         result(0) = 0;
    //         result(1) = 0;
    //         result(2) = 1;
    //         result(3) = 0;
    //         result(4) = 0;
    //     }
            
    //     else if( z_pm_ < dis && dis <= z_pm_end)
    //     {
    //         result(0) = 0;
    //         result(1) = 0;
    //         result(2) = fabs(1/(z_pm_end - z_pm_)*(dis - z_pm_end));
    //         result(3) = 1/(z_pb - z_pb_start)*(dis - z_pb_start);
    //         result(4) = 0;
    //     }
            
    //     else if( z_pb < dis && dis <= z_pb_)
    //     {
    //         result(0) = 0;
    //         result(1) = 0;
    //         result(2) = 0;
    //         result(3) = 1;
    //         result(4) = 0;
    //     }
            
    //     else if( z_pb_ < dis && dis <= z_pb_end)
    //     {
    //         result(0) = 0;
    //         result(1) = 0;
    //         result(2) = 0;
    //         result(3) = fabs(1/(z_pb_end - z_pb_)*(dis - z_pb_end));
    //         result(4) = 1/(z_pvb - z_pvb_start)*(dis - z_pvb_start);       
    //     }

    //     else
    //     {
    //         result(0) = 0;
    //         result(1) = 0;
    //         result(2) = 0;
    //         result(3) = 0;
    //         result(4) = 1;
    //     }
        
    //     // std::cout<<"dis_input:"<<result.transpose()<<std::endl;
    //     return result;
    // }
     static Eigen::Matrix<double, 5, 1> displacementInput(const double origin, const double z_dis)
    {
        //parameters fuzzy-logic-displacement function
        //--------------------
        double z_pvs = 0.0005;
        double z_pvs_end = 0.00175;
        //--------------------
        
        double z_ps = 0.003;
        double z_ps_end = 0.004;
        //--------------------
        
        double z_pm = 0.005;
        double z_pm_end = 0.0075;
        //--------------------

        double z_pb = 0.01;
        double z_pb_end = 0.012;
        //--------------------
        
        double z_pvb = 0.013;
        
        double dis;
        // std::cout<<"spiral orign: "<<origin<<std::endl;
        Eigen::Matrix<double, 5, 1> result;
        result.setZero();

        dis = origin - z_dis;
        
        // dis = 0.0044; //debug
        // dis = 0.0059;
        if( dis <= z_pvs)
        {
            result(0) = 1; // move
            result(1) = 0; // fake
            result(2) = 0; // shallow dead
            result(3) = 0; // hole
            result(4) = 0; // deep dead
        }
            
        else if( z_pvs < dis && dis <= z_pvs_end)
        {
            result(0) = fabs(1/(z_pvs_end - z_pvs)*(dis - z_pvs_end));
            result(1) = 1/(z_ps - z_pvs)*(dis - z_pvs);
            result(2) = 0;
            result(3) = 0;
            result(4) = 0;
        }
            
        else if( z_pvs_end < dis && dis <= z_ps)
        {
            result(0) = 0;
            result(1) = 1/(z_ps - z_pvs)*(dis - z_pvs);;
            result(2) = 0;
            result(3) = 0;
            result(4) = 0;
        }
            
        else if( z_ps < dis && dis <= z_ps_end)
        {
            result(0) = 0;
            result(1) = fabs(1/(z_ps_end - z_ps)*(dis - z_ps_end));
            result(2) = 1/(z_pm - z_ps)*(dis - z_ps);
            result(3) = 0;
            result(4) = 0;
        }
            
        else if( z_ps_end < dis && dis <= z_pm)
        {
            result(0) = 0;
            result(1) = 0;
            result(2) = 1/(z_pm - z_ps)*(dis - z_ps);
            result(3) = 0;
            result(4) = 0;
        }
            
        else if( z_pm < dis && dis <= z_pm_end)
        {
            result(0) = 0;
            result(1) = 0;
            result(2) = fabs(1/(z_pm_end - z_pm)*(dis - z_pm_end));
            result(3) = 1/(z_pb - z_pm)*(dis - z_pm);
            result(4) = 0;
        }
            
        else if( z_pm_end < dis && dis <= z_pb)
        {
            result(0) = 0;
            result(1) = 0;
            result(2) = 0;
            result(3) = 1/(z_pb - z_pm)*(dis - z_pm);
            result(4) = 0;
        }
            
        else if( z_pb < dis && dis <= z_pb_end)
        {
            result(0) = 0;
            result(1) = 0;
            result(2) = 0;
            result(3) = fabs(1/(z_pb_end - z_pb)*(dis - z_pb_end));
            result(4) = 1/(z_pvb - z_pb)*(dis - z_pb);       
        }
        
        else if( z_pb_end < dis && dis <= z_pvb)
        {
            result(0) = 0;
            result(1) = 0;
            result(2) = 0;
            result(3) = 0;
            result(4) = 1/(z_pvb - z_pb)*(dis - z_pb);     
        }

        else
        {
            result(0) = 0;
            result(1) = 0;
            result(2) = 0;
            result(3) = 0;
            result(4) = 1;
        }
        
        // std::cout<<"dis_input:"<<result.transpose()<<std::endl;

        // std::cout<<"fuzzified displacement: "<<result.transpose()<<std::endl;
        return result;
    }

    // static Eigen::Vector3d forceInput(const double force)
    // {
    //     //parameters fuzzy-logic-force function
    //     //--------------------
    //     double f_ps = 8.0;
    //     double f_ps_end = 9.0;
    //     //--------------------
    //     double f_pm_start = 8.0;
    //     double f_pm = 9.0;
    //     double f_pm_ = 18.0;
    //     double f_pm_end = 26.0;
    //     //--------------------
    //     double f_pb_start = 26.0;
    //     double f_pb = 28.0;
    //     //--------------------

    //     Eigen::Vector3d result;
    //     result.setZero();

    //     if( force <= f_ps)
    //     {
    //         result(0) = 1;
    //         result(1) = 0;
    //         result(2) = 0;
    //     }
            
    //     else if( f_ps < force && force <= f_ps_end)
    //     {
    //         result(0) = fabs(1/(f_ps_end-f_ps)*(force - f_ps_end));
    //         result(1) = 1/(f_pm-f_pm_start)*(force - f_pm_start);
    //         result(2) = 0;
    //     }
            
    //     else if( f_pm < force && force <= f_pm_)
    //     {
    //         result(0) = 0;
    //         result(1) = 1;
    //         result(2) = 0;    
    //     }
            
    //     else if( f_pm_ < force && force <= f_pm_end)
    //     {
    //         result(0) = 0;
    //         result(1) = fabs(1/(f_pm_end-f_pm_)*(force - f_pm_end));
    //         result(2) = 1/(f_pb-f_pb_start)*(force-f_pb_start);
    //     }
            
    //     else
    //     {
    //         result(0) = 0;
    //         result(1) = 0;
    //         result(2) = 1;    
    //     }
        
    //     // std::cout<<"force_input:"<<result.transpose()<<std::endl;
    //     return result;
    // }

    static Eigen::Vector3d forceInput(const double force)
    {
        //parameters fuzzy-logic-force function
        //--------------------
        double f_ps = 2.0;
        double f_ps_end = 8.0;
        //--------------------
        
        double f_pm = 16.0;
        double f_pm_end = 23.0;
        //--------------------
        
        double f_pb = 30.0;
        //--------------------

        double f = force;

        Eigen::Vector3d result;
        result.setZero();

        // f = 1.4217; //debug
        // f = 0.6447;
        if( f <= f_ps)
        {
            result(0) = 1;
            result(1) = 0;
            result(2) = 0;
        }
            
        else if( f_ps < f && f <= f_ps_end)
        {
            result(0) = fabs(1/(f_ps_end-f_ps)*(f - f_ps_end));
            result(1) = 1/(f_pm-f_ps)*(f - f_ps);
            result(2) = 0;
        }
            
        else if( f_ps_end < f && f <= f_pm)
        {
            result(0) = 0;
            result(1) = 1/(f_pm-f_ps)*(f - f_ps);
            result(2) = 0;    
        }
            
        else if( f_pm < f && f <= f_pm_end)
        {
            result(0) = 0;
            result(1) = fabs(1/(f_pm_end - f_pm)*(f - f_pm_end));
            result(2) = 1/(f_pb - f_pm)*(f - f_pm);
        }

        else if( f_pm_end < f && f <= f_pb)
        {
            result(0) = 0;
            result(1) = 0;
            result(2) = 1/(f_pb - f_pm)*(f - f_pm);
        }
            
        else
        {
            result(0) = 0;
            result(1) = 0;
            result(2) = 1;    
        }
        
        // std::cout<<"force_input:"<<result.transpose()<<std::endl;

        // std::cout<<"Fuzzified force: "<<result.transpose()<<std::endl;
        return result;
    }

    static double andOperator(const double x, const double y, const double z)
    {   
        double result;

        if( x <= y)
        {
            if(x <= z)  result = x;
            else    result = z;
        }            
            
        else
        {
            if(y <= z)  result = y;
            else    result = z;
        }
         
        return result;     
    }
    
    
    static double fuzzyOutput(const Eigen::Vector2d v, const Eigen::Matrix<double, 5, 1> z, const Eigen::Vector3d f)
    {
        int v_size = 2;
        int z_size = 5;
        int f_size = 3;

        double num = 0.0;
        double den = 0.0;
        double acceptable = 0.2;
        
        double result; 

        Eigen::MatrixXd output_set;
        Eigen::MatrixXd state_set;

        output_set.resize(v_size*z_size*f_size,1);
        output_set.setZero();

        state_set.resize(v_size*z_size*f_size,1);
        output_set.setZero();
        
        // state_set << MOVE, MOVE, HOLE, MOVE, MOVE, FAKE, DEAD, HOLE, HOLE, HOLE, HOLE, HOLE, DEAD, DEAD, DEAD,
        //                 MOVE, MOVE, MOVE, MOVE, MOVE, MOVE, MOVE, MOVE, MOVE, MOVE, MOVE, MOVE, DEAD, DEAD, DEAD;
        state_set << CS_ONE, CS_TWO, CS_TWO, CS_ONE, CS_TWO, CS_THREE, CS_FIVE_ONE, CS_FIVE_ONE, CS_FOUR, CS_FOUR, CS_FOUR, CS_FOUR, CS_FIVE_TWO, CS_FIVE_TWO, CS_FIVE_TWO,
                        NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, NONE, CS_FIVE_TWO, CS_FIVE_TWO, CS_FIVE_TWO;
                            
        for(int k = 0; k < v_size; k++)
        {
            for(int i = 0; i < z_size; i++)
            {
                for(int j = 0; j < f_size; j++)
                {
                    output_set((z_size*f_size*(k)+f_size*(i)+j)) = andOperator(v(k),z(i),f(j));
                }                     
            }                 
        }

        for(int i = 0; i < z_size*f_size*v_size; i ++)
        {
            // if(state_set(i) != NONE)
            // {
                num = num + state_set(i)*output_set(i);   
                den = den + output_set(i);
                // if(den == 0)    result = 0;
                // else result = num/den;
                result = num/den;
            // }
        }

        if(fabs(result - NONE) <= acceptable)   result = NONE;
        else if(fabs(result - CS_ONE) <= acceptable)  result = CS_ONE;
        else if(fabs(result - CS_TWO) <= acceptable)  result = CS_TWO;
        else if(fabs(result - CS_THREE) <= acceptable)  result = CS_THREE;
        else if(fabs(result - CS_FOUR) <= acceptable)  result = CS_FOUR;
        else if(fabs(result - CS_FIVE_ONE) <= acceptable)  result = CS_FIVE_ONE;
        else if(fabs(result - CS_FIVE_TWO) <= acceptable)  result = CS_FIVE_TWO;
        else result = result;

        std::cout<<"result: "<<result<<std::endl;
        return result;
        
    }


};

#endif // !FUZZYCONTROL_H
