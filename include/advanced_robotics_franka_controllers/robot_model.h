#ifndef __ROBOT_MODEL__
#define __ROBOT_MODEL__

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_config.h>

#include <iostream>
#include <memory>
#include <fstream>

using namespace RigidBodyDynamics;
using namespace Eigen;
using namespace std;
#define dof 7
typedef Transform<double, 3, Eigen::Affine> Transform3d;


class RobotModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RobotModel();
    ~RobotModel();

    void getUpdateKinematics(const VectorXd &q, const VectorXd &qdot);

    const MatrixXd &getJacobian(const int &frame_id, Vector3d& tip_pos)
    {
        Jacobian(frame_id, tip_pos);
        return m_J_;
    }
    const Vector3d &getPosition(const int &frame_id, Vector3d& tip_pos)
    {
        Position(frame_id, tip_pos);
        return m_pos_;
    }
    const Matrix3d &getOrientation(const int &frame_id)
    {
        Orientation(frame_id);
        return m_Ori_;
    }
    const Transform3d &getTransformation(const int &frame_id, Vector3d& tip_pos)
    {
        Transformation(frame_id, tip_pos);
        return m_Trans_;
    }

private:
    void Jacobian(const int &frame_id, Vector3d& tip_pos);
    void Position(const int &frame_id, Vector3d& tip_pos);
    void Orientation(const int &frame_id);
    void Transformation(const int &frame_id, Vector3d& tip_pos);
	void setRobot();

    /////////////////////////////////////////////////////////////////
    shared_ptr<Model> model_;
    Body body_[dof];
    Body base_;

    Joint joint_[dof];

    VectorXd q_;
    VectorXd qdot_;

    VectorXd q_real_;
    VectorXd qdot_real_;

    double mass_[dof];
    Math::Vector3d axis_[dof];
    Math::Vector3d inertia_[dof];
    Math::Vector3d joint_position_global_[dof];
    Math::Vector3d joint_position_local_[dof];
    Math::Vector3d com_position_[dof];


    Math::VectorNd q_rbdl_;
    Math::VectorNd qdot_rbdl_;
    Math::VectorNd qddot_rbdl_;
    Math::VectorNd tau_;

    unsigned int body_id_[dof];

    Vector3d m_pos_;
    Matrix3d m_Ori_;
    MatrixXd m_J_;

    Transform3d m_Trans_;
    Transform3d m_base_;

    int m_robot_type_;


protected:
    unsigned int m_nv_;
    unsigned int m_na_;
    unsigned int m_nq_;
};

#endif