#ifndef CRISPCONTROL_H
#define CRISPCONTROL_H

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

namespace CrispLogic
{
    static int velocityInputCrisp(const double z_vel)
    {
        int result = 0;
        double v;
        v = 0.0065;

        if(z_vel <= v)
        {
            result = 1;
        }
        else
        {
            result = 2;
        }

        return result;
        
    }

     static int displacementInputCrisp(const double origin, const double z_dis)
    {
        double z_pvs, z_ps, z_pm, z_pb;
        z_pvs = 0.0005;
        z_ps = 0.003;
        z_pm = 0.005;
        z_pb = 0.01;

        int result = 0;
        double dis;

        dis = origin - z_dis;
        
        if(dis <= z_pvs)    result = 1;
        else if(z_pvs < dis && dis <= z_ps) result = 2;
        else if(z_ps < dis && dis <= z_pm) result = 3;
        else if(z_pm < dis && dis <= z_pb) result = 4;
        else result = 5;

        return result;
    }

    
    static int forceInputCrisp(const double force)
    {
     
        double f_ps, f_pm;
        f_ps = 6.2;
        f_pm = 20.6667;


        int result;
        
        if(force <= f_ps) result = 1;
        else if(f_ps < force && force <= f_pm) result = 2;
        else result = 3;

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
    
    
    static int crispOutput(const int v, const int z, const int f)
    {
        int result;    

        if(v == 1)
        {
            if(z == 1)
            {
                if(f == 1)      result = CS_ONE;
                else if(f == 2) result = CS_TWO;
                else            result = CS_TWO;
            }
            else if(z == 2)
            {
                if(f == 1)      result = CS_ONE;
                else if(f == 2) result = CS_TWO;
                else            result = CS_THREE;
            }
            else if(z == 3)
            {
                if(f == 1)      result = CS_FIVE_ONE;
                else if(f == 2) result = CS_FIVE_ONE;
                else            result = CS_FOUR;
            }
            else if(z == 4)
            {
                if(f == 1)      result = CS_FOUR;
                else if(f == 2) result = CS_FOUR;
                else            result = CS_FOUR;
            }
            else                result = CS_FIVE_ONE; 
        }
        else
        {
            if(z == 5) result = CS_FIVE_TWO;
            else result = NONE;
        }
                

        return result;
        
    }


};

#endif // !CRISPCONTROL_H
