#ifndef MATH_TYPE_DEFINE_H
#define MATH_TYPE_DEFINE_H


#define DEG2RAD (0.01745329251994329576923690768489)
#define ZCE (1e-5)
// constexpr size_t MAX_DOF=50;

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <fstream>

#define GRAVITY 9.80665
#define MAX_DOF 50U
#define RAD2DEG 1/DEG2RAD

namespace Eigen
{

// Eigen default type definition
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)   \
  typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix;  \
  typedef Matrix<Type, Size, 1>    Vector##SizeSuffix##TypeSuffix;  \
  typedef Matrix<Type, 1, Size>    RowVector##SizeSuffix##TypeSuffix;

typedef double	rScalar;

EIGEN_MAKE_TYPEDEFS(rScalar, d, 5, 5)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 6, 6)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 7, 7)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 8, 8)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 12, 12)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 18, 18)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 28, 28)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 30, 30)
EIGEN_MAKE_TYPEDEFS(rScalar, d, 32, 32)

// typedef Transform<rScalar, 3, Eigen::Isometry> HTransform;  // typedef Transform< double, 3, Isometry > 	Eigen::Isometry3d

typedef Matrix<rScalar, 1, 3>	Matrix1x3d;
typedef Matrix<rScalar, 1, 4>	Matrix1x4d;
typedef Matrix<rScalar, 4, 3>	Matrix4x3d;
typedef Matrix<rScalar, 6, 3>	Matrix6x3d;
typedef Matrix<rScalar, 6, 7>	Matrix6x7d;
typedef Matrix<rScalar, 8, 4>	Matrix8x4d;
typedef Matrix<rScalar, 8, 2>	Matrix8x2d;
typedef Matrix<rScalar, -1, 1, 0, MAX_DOF, 1> VectorJXd;
typedef Matrix<rScalar, -1, 1, 0, 12, 1> VectorLXd; //Leg IK
typedef Matrix<rScalar, -1, -1, 0, MAX_DOF, MAX_DOF> MatrixJXd;

//Complex
typedef Matrix<std::complex<double>,8,4> Matrix8x4cd;

}

namespace DyrosMath
{
  //constexpr double GRAVITY {9.80665};
//constexpr double DEG2RAD {};

struct ImpedanceControllerParameter
{
  Eigen::VectorXd active_command_idx{6};
  Eigen::VectorXd parameter_M{6};
  Eigen::VectorXd parameter_D{6};
  Eigen::VectorXd parameter_K{6};
  Eigen::VectorXd desired_value{6};

  void initialize(){
    active_command_idx.setZero();
    parameter_M.setZero();
    parameter_D.setZero();
    parameter_K.setZero();
    desired_value.setZero();
  }
};

struct ImpedanceController
{
  double Ts = 0.001;
  Eigen::VectorXd errorValuePPrev{6};
  Eigen::VectorXd errorValuePrev{6};
  Eigen::VectorXd errorValue{6};

  Eigen::VectorXd impedancePPrev{6};
  Eigen::VectorXd impedancePrev{6};
  Eigen::VectorXd impedance{6};

  ImpedanceControllerParameter param;

  Eigen::VectorXd getImpedance()
  {
    return impedance;
  }

  void initialize(){
    errorValuePPrev.setZero();
    errorValuePrev.setZero();
    errorValue.setZero();
    impedancePPrev.setZero();
    impedancePrev.setZero();
    impedance.setZero();
    param.initialize();
  }

  void update(Eigen::VectorXd FValue){

    //update previous values
    errorValuePPrev = errorValuePrev;
    errorValuePrev = errorValue;
    errorValue = FValue - param.desired_value;

    impedancePPrev = impedancePrev;
    impedancePrev = impedance;
    
    // update impedance
    double d0,d1,d2,n0,n1,n2;
    for(int i=0; i<6; i++)
    {
      d0 = 4.0 * param.parameter_M(i) + 2.0 * Ts * param.parameter_D(i) + Ts * Ts * param.parameter_K(i);
      d1 = - 8.0 * param.parameter_M(i) + 2.0 * Ts * Ts * param.parameter_K(i);
      d2 = 4.0 * param.parameter_M(i) - 2.0 * Ts * param.parameter_D(i) + Ts * Ts * param.parameter_K(i);

      n0 = Ts*Ts;
      n1 = 2.0*Ts*Ts;
      n2 = Ts*Ts;

      if (d0 > 1e-8)
        impedance(i) = param.active_command_idx(i) * (n0*errorValue(i) + n1*errorValuePrev(i) + n2*errorValuePPrev(i) - d1*impedancePrev(i) - d2*impedancePPrev(i))/d0;
      else
        impedance(i) = 0.0;

      //  예외처리
      // if(isnan(impedance(i)) || isinf(impedance(i)))
      //   impedance(i) = 0.0;    
    }
  }

};

struct SaveData
{
  FILE *fwrite;
  
  void open(std::string filename)
  {
    fwrite = fopen(filename.c_str(),"w");
  }
  // void save(Eigen::VectorXd input){
  //   for(int i=0; i<input.size(); i++)
  //     fprintf(fwrite, "%.15f\t", input(i));
  //   fprintf(fwrite, "\n");
  // }

  void save(double input){
    fprintf(fwrite, "%.9f\t \n", input);
  }

  void save(Eigen::MatrixXd input){
    for(int i=0; i<input.cols(); i++){
      for(int j=0; j<input.rows(); j++)
        fprintf(fwrite, "%.9f\t", input(j,i));
    }
    fprintf(fwrite, "\n");
  }

};

struct LowPassFilter2
{
  // second order low-pass filter
  double CutOffFreq, Ts;
  double zeta, Wn, Den0, Den1, Den2, Num2, Num1, Num0;
  bool b_initialized, b_update;
  // Eigen::Matrix<double, 7, 1> 
  Eigen::VectorXd RawDataPrev{7};
  Eigen::VectorXd RawDataPPrev{7};
  Eigen::VectorXd FilteredDataPrev{7};
  Eigen::VectorXd FilteredDataPPrev{7};

  LowPassFilter2(double dCutOffFreq, double dTs) // set cutoff-frequency and (Ts)control period
  : CutOffFreq(dCutOffFreq), Ts(dTs)
  {
    zeta = 0.9;
    Wn = dCutOffFreq*2.0*3.141592;
    b_initialized = false;
    b_update = false;

    RawDataPrev.setZero();
    RawDataPPrev.setZero();
    FilteredDataPrev.setZero();
    FilteredDataPPrev.setZero();

    Den0 = 4.0+4.0*zeta*Ts*Wn + Ts*Wn*Ts*Wn;
    Den1 = 2.0*Ts*Wn*Ts*Wn - 8.0;
    Den2 = 4.0-4.0*zeta*Ts*Wn + Ts*Wn*Ts*Wn;
    Num2 = Ts*Ts*Wn*Wn;
    Num1 = 2.0*Ts*Ts*Wn*Wn;
    Num0 = Ts*Ts*Wn*Wn;
  }

  void initialize(double dCutOffFreq, double dTs)
  {
    b_initialized = true;
    CutOffFreq = dCutOffFreq;
    Ts=dTs;
    zeta = 0.9;
    Wn = dCutOffFreq*2.0*3.141592;

    RawDataPrev.setZero();
    RawDataPPrev.setZero();
    FilteredDataPrev.setZero();
    FilteredDataPPrev.setZero();

    Den0 = 4.0+4.0*zeta*Ts*Wn + Ts*Wn*Ts*Wn;
    Den1 = 2.0*Ts*Wn*Ts*Wn - 8.0;
    Den2 = 4.0-4.0*zeta*Ts*Wn + Ts*Wn*Ts*Wn;
    Num2 = Ts*Ts*Wn*Wn;
    Num1 = 2.0*Ts*Ts*Wn*Wn;
    Num0 = Ts*Ts*Wn*Wn;
  }

  Eigen::VectorXd update(Eigen::VectorXd RawDataInput)
  {
    if(!b_update)
    {
      RawDataPrev = RawDataInput;
      RawDataPPrev = RawDataInput;
      FilteredDataPrev = RawDataInput;
      FilteredDataPPrev = RawDataInput;
      b_update = true;
    }

    if(b_initialized)
    { 
      Eigen::VectorXd FilteredDataReturn(7);
      FilteredDataReturn.setZero();

      for(int i=0; i<7; i++){
        FilteredDataReturn(i) = (Num0*RawDataInput(i) + Num1*RawDataPrev(i) + Num2*RawDataPPrev(i) - Den1*FilteredDataPrev(i) - Den2*FilteredDataPPrev(i))/Den0;    
        RawDataPPrev(i) = RawDataPrev(i); RawDataPrev(i) = RawDataInput(i); FilteredDataPPrev(i) = FilteredDataPrev(i); FilteredDataPrev(i) = FilteredDataReturn(i);
      }
      return FilteredDataReturn;
    }
    else
      return RawDataInput;
  }
};

static double minJerkTraj(double t, double &qinit, double &qtarget, double period)
{
	if (period <= ZCE || t<0)
		return qinit;
	else
	{
		//double q = qinit + (qtarget - qinit) * ( 10.*pow(t / period, 3) - 15.*pow(t / period, 4) + 6.*pow(t / period, 5) );
    double t_ = t/period;
    double q = qinit + (qtarget - qinit) * ( 10.*t_*t_*t_ - 15.*t_*t_*t_*t_ + 6.*t_*t_*t_*t_*t_ );
		if (t<=period)
			return q;
		else
			return qtarget;
	}
}

static double minJerkTrajVel(double t, double &qinit, double &qtarget, double period)
{
	if (period <= ZCE || t<0)
		return 0.;
	else
	{
		double q = (qtarget - qinit)*(30.*pow(t / period, 2) - 60.*pow(t / period, 3) + 30.*pow(t / period, 4))/ period;
		if (t<=period)
			return q;
		else
			return 0.;
	}
}

static double minJerkTrajAcc(double t, double &qinit, double &qtarget, double period)
{
	if (period <= ZCE || t<0)
		return 0.;
	else
	{
		double q = (qtarget - qinit)*(60.*(t / period) - 180.*pow(t / period, 2) + 120.*pow(t / period, 3))/ (period*period);
		if (t<=period)
			return q;
		else
			return 0.;
	}
}



static double cubic(double time,     ///< Current time
             double time_0,   ///< Start time
             double time_f,   ///< End time
             double x_0,      ///< Start state
             double x_f,      ///< End state
             double x_dot_0,  ///< Start state dot
             double x_dot_f   ///< End state dot
             )
{
  double x_t;

  if (time < time_0)
  {
    x_t = x_0;
  }
  else if (time > time_f)
  {
    x_t = x_f;
  }
  else
  {
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double total_time2 = total_time * total_time;  // pow(t,2)
    double total_time3 = total_time2 * total_time; // pow(t,3)
    double total_x    = x_f - x_0;

    x_t = x_0 + x_dot_0 * elapsed_time

        + (3 * total_x / total_time2
           - 2 * x_dot_0 / total_time
           - x_dot_f / total_time)
        * elapsed_time * elapsed_time

        + (-2 * total_x / total_time3 +
           (x_dot_0 + x_dot_f) / total_time2)
        * elapsed_time * elapsed_time * elapsed_time;
  }

  return x_t;
}

static double cubicDot(double time,     ///< Current time
             double time_0,   ///< Start time
             double time_f,   ///< End time
             double x_0,      ///< Start state
             double x_f,      ///< End state
             double x_dot_0,  ///< Start state dot
             double x_dot_f   ///< End state dot
             )
{
  double x_dot_t;

  if (time < time_0)
  {
    x_dot_t = x_dot_0;
  }
  else if (time > time_f)
  {
    x_dot_t = x_dot_f;
  }
  else
  {
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double total_time2 = total_time * total_time;  // pow(t,2)
    double total_time3 = total_time2 * total_time; // pow(t,3)
    double total_x    = x_f - x_0;

    x_dot_t = x_dot_0

        + 2*(3 * total_x / total_time2
           - 2 * x_dot_0 / total_time
           - x_dot_f / total_time)
        * elapsed_time

        + 3*(-2 * total_x / total_time3 +
           (x_dot_0 + x_dot_f) / total_time2)
        * elapsed_time * elapsed_time;
  }

  return x_dot_t;
}

static Eigen::Matrix<double, 2, 1> spiral(double time,     ///< Current time
	double time_0,   ///< Start time
	double time_f,   ///< End time
	Eigen::Matrix<double, 2, 1> x_0,      ///< Start state
	double line_v,
	double pitch
)
{
	// TODO: Modify this method to debug your code

	Eigen::Matrix<double, 2, 1> result_xy;

	if (time < time_0)
	{
		result_xy = x_0;
	}
	else if (time > time_f)
	{
		double total_time = time_f - time_0;

		double a = 0.0;
		double b = pitch / (2 * M_PI);


		double theta = sqrt(2 * line_v*total_time / b);
		double r = a + b * theta;

		result_xy(0) = x_0(0) + r * cos(theta);
		result_xy(1) = x_0(1) + r * sin(theta);
		
	}
	else
	{
		
		double elapsed_time = time - time_0;

		double a = 0.0;
		double b = pitch / (2 * M_PI);


		double theta = sqrt(2 * line_v*elapsed_time / b);
		double r = a + b * theta;

		result_xy(0) = x_0(0) + r * cos(theta);
		result_xy(1) = x_0(1) + r * sin(theta);

	}

	return result_xy;
}

static Eigen::Matrix<double, 2, 1> ellipseSpiral(double time,     ///< Current time
	double time_0,   ///< Start time
	double time_f,   ///< End time
	Eigen::Matrix<double, 2, 1> x_0,      ///< Start state
	double line_v,
	double pitch,
  const double n,
  const double m
)
{
	// TODO: Modify this method to debug your code

	Eigen::Matrix<double, 2, 1> result_xy;

	if (time < time_0)
	{
		result_xy = x_0;
	}
	else if (time > time_f)
	{
		double total_time = time_f - time_0;

		double a = 0.0;
		double b = pitch / (2 * M_PI);


		double theta = sqrt(2 * line_v*total_time / b);
		double r = a + b * theta;

		result_xy(0) = x_0(0) + n*r * cos(theta);
		result_xy(1) = x_0(1) + m*r * sin(theta);
		
	}
	else
	{
		
		double elapsed_time = time - time_0;

		double a = 0.0;
		double b = pitch / (2 * M_PI);


		double theta = sqrt(2 * line_v*elapsed_time / b);
		double r = a + b * theta;

		result_xy(0) = x_0(0) + r * n*cos(theta);
		result_xy(1) = x_0(1) + r * m*sin(theta);

	}

	return result_xy;
}

static const Eigen::Matrix3d skew(const Eigen::Vector3d &src)
{
    Eigen::Matrix3d skew;
    skew.setZero();
    skew(0, 1) = -src[2];
    skew(0, 2) = src[1];
    skew(1, 0) = src[2];
    skew(1, 2) = -src[0];
    skew(2, 0) = -src[1];
    skew(2, 1) = src[0];

    return skew;
}

template <int N>
static Eigen::Matrix<double, N, 1> cubicVector(double time,     ///< Current time
                                                double time_0,   ///< Start time
                                                double time_f,   ///< End time
                                                Eigen::Matrix<double, N, 1> x_0,      ///< Start state
                                                Eigen::Matrix<double, N, 1> x_f,      ///< End state
                                                Eigen::Matrix<double, N, 1> x_dot_0,  ///< Start state dot
                                                Eigen::Matrix<double, N, 1> x_dot_f   ///< End state dot
    )
{

  Eigen::Matrix<double, N, 1> res;
  for (unsigned int i=0; i<N; i++)
  {
    res(i) = cubic(time, time_0, time_f, x_0(i), x_f(i), x_dot_0(i), x_dot_f(i));
  }
  return res;
}

// Original Paper
// Kang, I. G., and F. C. Park.
// "Cubic spline algorithms for orientation interpolation."
// International journal for numerical methods in engineering 46.1 (1999): 45-64.
const static Eigen::Matrix3d rotationCubic(
    double time, double time_0, double time_f,
    const Eigen::Vector3d &w_0, const Eigen::Vector3d &a_0,
    const Eigen::Matrix3d &rotation_0, const Eigen::Matrix3d &rotation_f)
{
  Eigen::Matrix3d rot;
  Eigen::Matrix3d r_skew;
  r_skew = (rotation_0.transpose() * rotation_f).log();
  Eigen::Vector3d a, b, c, r;
  double tau = (time - time_0) / (time_f - time_0);
  r(0) = r_skew(2,1);
  r(1) = r_skew(0,2);
  r(2) = r_skew(1,0);
  c = w_0;
  b = a_0 / 2;
  a = r - b -c;
  rot = rotation_0 * (skew(a*pow(tau,3) + b*pow(tau,2) + c*tau)).exp();

  return rot;
}
const static Eigen::Matrix3d rotationCubic(double time,
                                     double time_0,
                                     double time_f,
                                     const Eigen::Matrix3d &rotation_0,
                                     const Eigen::Matrix3d &rotation_f)
{
  if(time >= time_f)
  {
    return rotation_f;
  }
  else if(time < time_0)
  {
    return rotation_0;
  }
  double tau = cubic(time,time_0,time_f,0,1,0,0);
  Eigen::Matrix3d rot_scaler_skew;
  rot_scaler_skew = (rotation_0.transpose() * rotation_f).log();
  //rot_scaler_skew = rot_scaler_skew.log();
  /*
  Eigen::Matrix3d rotation_exp;
  Eigen::Vector3d a1, b1, c1, r1;
  r1(0) = rotation_temp(2,1);
  r1(1) = rotation_temp(0,2);
  r1(2) = rotation_temp(1,0);
  c1.setZero(); // angular velocity at t0 --> Zero
  b1.setZero(); // angular acceleration at t0 --> Zero
  a1 = r1 - b1 - c1;
  //double tau = (time - time_0) / (time_f-time_0);
  double tau2 = tau*tau;
  double tau3 = tau2*tau;
  //Eigen::Vector3d exp_vector = (a1*tau3+b1*tau2+c1*tau);
  Eigen::Vector3d exp_vector = (a1*tau);
  rotation_exp.setZero();
  rotation_exp(0,1) = -exp_vector(2);
  rotation_exp(0,2) =  exp_vector(1);
  rotation_exp(1,0) =  exp_vector(2);
  rotation_exp(1,2) = -exp_vector(0);
  rotation_exp(2,0) = -exp_vector(1);
  rotation_exp(2,1) =  exp_vector(0);

  */
  //Eigen::Matrix3d result = rotation_0 * rotation_exp.exp();
  Eigen::Matrix3d result = rotation_0 * (rot_scaler_skew * tau).exp();

  return result;
}

static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                       Eigen::Matrix3d desired_rotation)
{
  Eigen::Vector3d phi;
  Eigen::Vector3d s[3], v[3], w[3];

  for (int i = 0; i < 3; i++) {
    v[i] = current_rotation.block<3, 1>(0, i);
    w[i] = desired_rotation.block<3, 1>(0, i);
    s[i] = v[i].cross(w[i]);
  }
  phi = s[0] + s[1] + s[2];
  phi = -0.5* phi;

  return phi;
}

static Eigen::Isometry3d multiplyIsometry3d(Eigen::Isometry3d A,
                                      Eigen::Isometry3d B)
{
  Eigen::Isometry3d AB;

  AB.linear() = A.linear()*B.linear();
  AB.translation() = A.linear()*B.translation() + A.translation();
  return AB;
}

static Eigen::Vector3d multiplyIsometry3dVector3d(Eigen::Isometry3d A,
                                      Eigen::Vector3d B)
{
  Eigen::Vector3d AB;
  AB = A.linear()*B + A.translation();
  return AB;
}

static Eigen::Isometry3d inverseIsometry3d(Eigen::Isometry3d A)
{
  Eigen::Isometry3d A_inv;

  A_inv.linear() = A.linear().transpose();
  A_inv.translation() = -A.linear().transpose()*A.translation();
  return A_inv;
}



static Eigen::Matrix3d rotateWithZ(double yaw_angle)
{
  Eigen::Matrix3d rotate_wth_z(3, 3);

  rotate_wth_z(0, 0) = cos(yaw_angle);
  rotate_wth_z(1, 0) = sin(yaw_angle);
  rotate_wth_z(2, 0) = 0.0;

  rotate_wth_z(0, 1) = -sin(yaw_angle);
  rotate_wth_z(1, 1) = cos(yaw_angle);
  rotate_wth_z(2, 1) = 0.0;

  rotate_wth_z(0, 2) = 0.0;
  rotate_wth_z(1, 2) = 0.0;
  rotate_wth_z(2, 2) = 1.0;

  return rotate_wth_z;
}

static Eigen::Matrix3d rotateWithY(double pitch_angle)
{
  Eigen::Matrix3d rotate_wth_y(3, 3);

  rotate_wth_y(0, 0) = cos(pitch_angle);
  rotate_wth_y(1, 0) = 0.0;
  rotate_wth_y(2, 0) = -sin(pitch_angle);

  rotate_wth_y(0, 1) = 0.0;
  rotate_wth_y(1, 1) = 1.0;
  rotate_wth_y(2, 1) = 0.0;

  rotate_wth_y(0, 2) = sin(pitch_angle);
  rotate_wth_y(1, 2) = 0.0;
  rotate_wth_y(2, 2) = cos(pitch_angle);

  return rotate_wth_y;
}

static Eigen::Matrix3d rotateWithX(double roll_angle)
{
  Eigen::Matrix3d rotate_wth_x(3, 3);

  rotate_wth_x(0, 0) = 1.0;
  rotate_wth_x(1, 0) = 0.0;
  rotate_wth_x(2, 0) = 0.0;

  rotate_wth_x(0, 1) = 0.0;
  rotate_wth_x(1, 1) = cos(roll_angle);
  rotate_wth_x(2, 1) = sin(roll_angle);

  rotate_wth_x(0, 2) = 0.0;
  rotate_wth_x(1, 2) = -sin(roll_angle);
  rotate_wth_x(2, 2) = cos(roll_angle);

  return rotate_wth_x;
}

static Eigen::Vector3d rot2Euler(Eigen::Matrix3d Rot)
{
    double beta;
    Eigen::Vector3d angle;
    beta = -asin(Rot(2,0));

    if(abs(beta) < 90*DEG2RAD)
        beta = beta;
    else
        beta = 180*DEG2RAD-beta;

    angle(0) = atan2(Rot(2,1),Rot(2,2)+1E-37); //roll
    angle(2) = atan2(Rot(1,0),Rot(0,0)+1E-37); //pitch
    angle(1) = beta; //yaw

    return angle;
}

static Eigen::Matrix3d Euler2rot(Eigen::Vector3d ang)
{
    Eigen::Matrix3d ROT;

    ROT = rotateWithZ(ang(2))*rotateWithY(ang(1))*rotateWithX(ang(0));

    return ROT;
}

static Eigen::Matrix3d angleaxis2rot(Eigen::Vector3d axis_angle_vector, double axis_angle)
{
  Eigen::Matrix3d ROT;
  double c = cos(axis_angle);
  double s = sin(axis_angle);
  double v = 1-c;

  ROT(0,0) = axis_angle_vector(0)*axis_angle_vector(0)*v + c;
  ROT(0,1) = axis_angle_vector(0)*axis_angle_vector(1)*v - axis_angle_vector(2)*s;
  ROT(0,2) = axis_angle_vector(0)*axis_angle_vector(2)*v + axis_angle_vector(1)*s;
  ROT(1,0) = axis_angle_vector(0)*axis_angle_vector(1)*v + axis_angle_vector(2)*s;
  ROT(1,1) = axis_angle_vector(1)*axis_angle_vector(1)*v + c;
  ROT(1,2) = axis_angle_vector(1)*axis_angle_vector(2)*v - axis_angle_vector(0)*s;
  ROT(2,0) = axis_angle_vector(0)*axis_angle_vector(2)*v - axis_angle_vector(1)*s;
  ROT(2,1) = axis_angle_vector(1)*axis_angle_vector(2)*v + axis_angle_vector(0)*s;
  ROT(2,2) = axis_angle_vector(2)*axis_angle_vector(2)*v + c;

  return ROT;
}

template <typename _Matrix_Type_>
_Matrix_Type_ pinv(const _Matrix_Type_ &a, double epsilon =std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);

    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}


static void floatGyroframe(Eigen::Isometry3d trunk, Eigen::Isometry3d reference, Eigen::Isometry3d new_trunk)
{
  Eigen::Vector3d rpy_ang;
  rpy_ang = DyrosMath::rot2Euler(reference.linear());

  Eigen::Matrix3d temp;
  temp = DyrosMath::rotateWithZ(-rpy_ang(2));

  new_trunk.linear() = temp*trunk.linear();
  new_trunk.translation() = temp*(trunk.translation() - reference.translation());
}

static Eigen::MatrixXd discreteRiccatiEquation(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd r, Eigen::MatrixXd q)
{
  int n=a.rows(); //number of rows
  int	m=b.cols(); //number of columns

  Eigen::MatrixXd z11(n, n), z12(n, n), z21(n, n), z22(n, n);

  z11 = a.inverse();
  z12 = a.inverse()*b*r.inverse()*b.transpose();
  z21 = q*a.inverse();
  z22 = a.transpose() + q*a.inverse()*b*r.inverse()*b.transpose();

  Eigen::MatrixXd z; z.resize(2*n, 2*n);
  z.setZero();
  z.topLeftCorner(n,n) = z11;
  z.topRightCorner(n,n) = z12;
  z.bottomLeftCorner(n,n) = z21;
  z.bottomRightCorner(n,n) = z22;


  std::vector<Eigen::VectorXd> eigVec_real(2*n);
  std::vector<Eigen::VectorXd> eigVec_img(2*n);

  for(int i=0; i<8; i++)
  {
    eigVec_real[i].resize(2*n);
    eigVec_real[i].setZero();
    eigVec_img[i].resize(2*n);
    eigVec_img[i].setZero();
  }

  Eigen::VectorXd deigVal_real(2*n);
  Eigen::VectorXd deigVal_img(2*n);
  deigVal_real.setZero();
  deigVal_img.setZero();
  Eigen::MatrixXd deigVec_real(2*n,2*n);
  Eigen::MatrixXd deigVec_img(2*n,2*n);
  deigVec_real.setZero();
  deigVec_img.setZero();

  deigVal_real = z.eigenvalues().real();
  deigVal_img = z.eigenvalues().imag();

  Eigen::EigenSolver<Eigen::MatrixXd> ev(z);
  //EigenVector Solver
  //Matrix3D ones = Matrix3D::Ones(3,3);
  //EigenSolver<Matrix3D> ev(ones);
  //cout << "The first eigenvector of the 3x3 matrix of ones is:" << endl << ev.eigenvectors().col(1) << endl;

  for(int i=0;i<2*n; i++)
  {
    for(int j=0; j<2*n; j++)
    {
      deigVec_real(j,i) = ev.eigenvectors().col(i)(j).real();
      deigVec_img(j,i) = ev.eigenvectors().col(i)(j).imag();
    }
  }

  //Order the eigenvectors
  //move e-vectors correspnding to e-value outside the unite circle to the left

  Eigen::MatrixXd tempZ_real(2*n, n), tempZ_img(2*n, n);
  tempZ_real.setZero();
  tempZ_img.setZero();
  int c=0;

  for (int i=0;i<2*n;i++)
  {
    if ((deigVal_real(i)*deigVal_real(i)+deigVal_img(i)*deigVal_img(i))>1.0) //outside the unit cycle
    {
      for(int j=0; j<2*n; j++)
      {
        tempZ_real(j,c) = deigVec_real(j,i);
        tempZ_img(j,c) = deigVec_img(j,i);
      }
      c++;
    }
  }

  Eigen::MatrixXcd tempZ_comp(2*n, n);
  for(int i=0;i<2*n;i++)
  {
    for(int j=0;j<n;j++)
    {
      tempZ_comp.real()(i,j) = tempZ_real(i,j);
      tempZ_comp.imag()(i,j) = tempZ_img(i,j);
    }
  }

  Eigen::MatrixXcd U11(n, n), U21(n, n), X(n, n);
  for(int i=0;i<n;i++)
  {
    for(int j=0;j<n;j++)
    {
      U11(i,j) = tempZ_comp(i,j);
      U21(i,j) = tempZ_comp(i+n,j);
    }
  }
  X = U21*(U11.inverse());

  Eigen::MatrixXd X_sol(n, n);
  for(int i=0;i<n;i++)
  {
    for(int j=0;j<n;j++)
    {
      X_sol(i,j) = X.real()(i,j);
    }
  }

  return X_sol;
}

static Eigen::Vector3d legGetPhi(Eigen::Isometry3d rotation_matrix1, Eigen::Isometry3d active_r1, Eigen::Vector6d ctrl_pos_ori)
{
   Eigen::Matrix3d active_r, rotation_matrix, x_rot, y_rot, z_rot, d_rot, s1_skew, s2_skew, s3_skew;
   x_rot.setZero();
   y_rot.setZero();
   z_rot.setZero();
   d_rot.setZero();
   s1_skew.setZero();
   s2_skew.setZero();
   s3_skew.setZero();

   active_r = active_r1.linear();

   x_rot=rotateWithX(ctrl_pos_ori(3));
   y_rot=rotateWithY(ctrl_pos_ori(4));
   z_rot=rotateWithZ(ctrl_pos_ori(5));
   d_rot=active_r.inverse()*z_rot*y_rot*x_rot;

   rotation_matrix = active_r.inverse() * rotation_matrix1.linear();

   s1_skew=skew(rotation_matrix.col(0));
   s2_skew=skew(rotation_matrix.col(1));
   s3_skew=skew(rotation_matrix.col(2));

   Eigen::Vector3d s1f, s2f, s3f, phi;
   s1f.setZero();
   s2f.setZero();
   s3f.setZero();

   s1f = s1_skew * d_rot.col(0);
   s2f = s2_skew * d_rot.col(1);
   s3f = s3_skew * d_rot.col(2);

   phi = (s1f + s2f + s3f) * (-1.0/2.0);

  return phi;
}
static void toEulerAngle(double qx, double qy, double qz, double qw, double& roll, double& pitch, double& yaw)
{
  double sinr = +2.0*(qw * qx + qy * qz);
  double cosr = +1.0-2.0*(qx * qx + qy * qy);
  roll = atan2(sinr,cosr);

  double sinp = +2.0*(qw * qy - qz * qx);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI/2, sinp);
  else
    pitch = asin(sinp);

  double siny = +2.0*(qw * qz + qx * qy);
  double cosy = +1.0-2.0*(qy * qy + qz * qz);
  yaw = atan2(siny, cosy);

}
static Eigen::Vector3d QuinticSpline(
                   double time,       ///< Current time
                   double time_0,     ///< Start time
                   double time_f,     ///< End time
                   double x_0,        ///< Start state
                   double x_dot_0,    ///< Start state dot
                   double x_ddot_0,   ///< Start state ddot
                   double x_f,        ///< End state
                   double x_dot_f,    ///< End state
                   double x_ddot_f )  ///< End state ddot
{
  double a1,a2,a3,a4,a5,a6;
  double time_s;

  Eigen::Vector3d result;

  if(time < time_0)
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
  a1=x_0;
  a2=x_dot_0;
  a3=x_ddot_0/2.0;

  Eigen::Matrix3d Temp;
  Temp<<pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
        3.0 * pow(time_s, 2), 4.0 * pow(time_s, 3), 5.0 * pow(time_s, 4),
        6.0 * time_s, 12.0 * pow(time_s, 2), 20.0 * pow(time_s, 3);

  Eigen::Vector3d R_temp;
  R_temp<<x_f-x_0-x_dot_0*time_s-x_ddot_0*pow(time_s,2)/2.0,
        x_dot_f-x_dot_0-x_ddot_0*time_s,
        x_ddot_f-x_ddot_0;

  Eigen::Vector3d RES;

  RES = Temp.inverse()*R_temp;

  a4=RES(0);
  a5=RES(1);
  a6=RES(2);

  double time_fs = time - time_0;

  double position = a1+a2*pow(time_fs,1)+a3*pow(time_fs,2)+a4*pow(time_fs,3)+a5*pow(time_fs,4)+a6*pow(time_fs,5);
  double velocity = a2+2.0*a3*pow(time_fs,1)+3.0*a4*pow(time_fs,2)+4.0*a5*pow(time_fs,3)+5.0*a6*pow(time_fs,4);
  double acceleration =2.0*a3+6.0*a4*pow(time_fs,1)+12.0*a5*pow(time_fs,2)+20.0*a6*pow(time_fs,3);


  result<<position,velocity,acceleration;

  return result;
}

static inline double lowPassFilter(double input, double prev, double ts, double tau)
{
    return (tau*prev + ts*input)/(tau+ts);
}
template <int N>
static Eigen::Matrix<double, N, 1> lowPassFilter(Eigen::Matrix<double, N, 1> input, Eigen::Matrix<double, N, 1> prev, double ts, double tau)
{
  Eigen::Matrix<double, N, 1> res;
  for(int i=0; i<N; i++)
  {
    res(i) = lowPassFilter(input(i), prev(i), ts, tau);
  }
  return res;
}

static Eigen::Vector4d rot2quat(const Eigen::Matrix3d rotation_M)
{
  double m00 = rotation_M(0,0);
  double m01 = rotation_M(0,1);
  double m02 = rotation_M(0,2);
  double m10 = rotation_M(1,0);
  double m11 = rotation_M(1,1);
  double m12 = rotation_M(1,2);
  double m20 = rotation_M(2,0);
  double m21 = rotation_M(2,1);
  double m22 = rotation_M(2,2);

  
  double qw, qx, qy, qz;

  float tr = m00 + m11 + m22;

  Eigen::Vector4d quat;

  quat.setZero();

  if (tr > 0)
  { 
    float S = sqrt(tr+1.0) * 2; // S=4*qw 
    qw = 0.25 * S;
    qx = (m21 - m12) / S;
    qy = (m02 - m20) / S; 
    qz = (m10 - m01) / S; 
  }
  
  else if((m00 > m11)&(m00 > m22))
  { 
    float S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
    qw = (m21 - m12) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S; 
    qz = (m02 + m20) / S; 
  }
  else if(m11 > m22)
  { 
    float S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S; 
    qy = 0.25 * S;
    qz = (m12 + m21) / S; 
  }
  else
  { 
    float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
    qw = (m10 - m01) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  }

  quat << qw, qx, qy, qz;

  return quat;
}

static Eigen::Matrix3d quat2Rot(const Eigen::Vector4d quat)
{
  Eigen::Matrix3d rot;
  double q0, q1, q2, q3;

  q0 = quat(3);
  q1 = quat(0);
  q2 = quat(1);
  q3 = quat(2);

  rot(0,0) = 1 - 2*(pow(q2,2)+pow(q3,2));
  rot(0,1) = 2*(q1*q2 - q0*q3);
  rot(0,2) = 2*(q0*q2 + q1*q3);
  rot(1,0) = 2*(q1*q2 + q0*q3);
  rot(1,1) = 1 - 2*(pow(q1,2) + pow(q3,2));
  rot(1,2) = 2*(q2*q3 - q0*q1);
  rot(2,0) = 2*(-q0*q2 + q1*q3);
  rot(2,1) = 2*(q2*q3 + q0*q1);
  rot(2,2) = 1 - 2*(pow(q1,2) + pow(q2,2));

  return rot;
}

static Eigen::MatrixXd leastSquareLinear(const std::vector<double> vec, const int interval)
{
  double total_size;
  double sub_size;

  Eigen::VectorXd a11,a12,a21,a22;
  Eigen::VectorXd b11,b21;

  Eigen::MatrixXd x,y;
  Eigen::MatrixXd coeff_vec;

  sub_size = floor(vec.size()/interval);
  total_size = sub_size*interval;
  
  a11.resize(interval);
  a12.resize(interval);
  a21.resize(interval);
  a22.resize(interval);
  b11.resize(interval);
  b21.resize(interval);

  x.resize(interval,sub_size);
  y.resize(interval,sub_size); 
  coeff_vec.resize(2,interval); //y = ax + b

  a11.setZero();
  a12.setZero();
  a21.setZero();
  a22.setZero();
  b11.setZero();
  b21.setZero();
  
  x.setZero();
  y.setZero();
  coeff_vec.setZero();

  for(int i = 0; i < interval; i++)
  {
    for(int j = 0; j < sub_size; j++)
    {
      x(i,j) = (i*sub_size + j)/1000;
      y(i,j) = vec[i*sub_size + j];      
    }
  }

  for(int i = 0; i < interval; i ++)
  {
    for(int j = 0; j < sub_size; j++)
    {
      a11(i) += x(i,j)*x(i,j);
      a12(i) += x(i,j);
      a21(i) += x(i,j);
      a22(i) += 1;
      b11(i) += x(i,j)*y(i,j);
      b21(i) += y(i,j);
    }
  }

  Eigen::Matrix2d temp_A;
  Eigen::Vector2d temp_B;
  
  for(int i = 0; i < interval; i ++)
  {
    temp_A << a11(i),a12(i),a21(i),a22(i);
    temp_B << b11(i),b21(i);

    coeff_vec.col(i) = temp_A.inverse()*temp_B;
    temp_A.setZero();
    temp_B.setZero();

  }

  std::cout<<"sub size: "<<sub_size<<std::endl;
  std::cout<<"total size: "<<total_size<<std::endl;
  // std::cout<<"a11: "<<a11.transpose()<<std::endl;
  // std::cout<<"a12: "<<a12.transpose()<<std::endl;
  // std::cout<<"a21: "<<a21.transpose()<<std::endl;
  // std::cout<<"a22: "<<a22.transpose()<<std::endl;
  // std::cout<<"b11: "<<b11.transpose()<<std::endl;
  // std::cout<<"b21: "<<b21.transpose()<<std::endl;
  
  return coeff_vec;

}

}
#endif

