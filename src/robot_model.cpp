#include <advanced_robotics_franka_controllers/robot_model.h>
#include <vector>

using namespace std;

RobotModel::RobotModel() {
	// model_ = new Model();
	// model_->gravity = Eigen::Vector3d(0., 0., -9.81);


	q_rbdl_.resize(dof);
	qdot_rbdl_.resize(dof);
	q_rbdl_.setZero();
	qdot_rbdl_.setZero();
	qddot_rbdl_.resize(m_na_ + 5);
	qddot_rbdl_.setZero();

	m_J_.resize(6, m_na_ + 2); // 
	m_J_.setZero();

	m_Ori_.resize(3, 3);
	m_pos_.setZero();
	m_Ori_.setZero();
	m_Trans_.linear().setZero();
	m_Trans_.translation().setZero();

	q_real_.resize(m_nv_);
	q_real_.setZero();
	qdot_real_.resize(m_nv_);
	qdot_real_.setZero();

	setRobot();
}
RobotModel::~RobotModel() {

}

void RobotModel::setRobot() {

	model_ = make_shared<Model>();
	model_->gravity = Vector3d(0., 0, -9.81);

///////// Manipulator //////////
	for (int i = 0; i < dof; i++) {
		mass_[i] = 1.0;
		inertia_[i] = Vector3d(0.001, 0.001, 0.001);
	}

	axis_[0] = Eigen::Vector3d::UnitZ();
	axis_[1] = Eigen::Vector3d::UnitY();
	axis_[2] = Eigen::Vector3d::UnitZ();
	axis_[3] = -1.0*Eigen::Vector3d::UnitY();
	axis_[4] = Eigen::Vector3d::UnitZ();
	axis_[5] = -1.0*Eigen::Vector3d::UnitY();
	axis_[6] = -Eigen::Vector3d::UnitZ();

// Only manipulator //
	joint_position_global_[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	joint_position_global_[1] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	joint_position_global_[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	joint_position_global_[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	joint_position_global_[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	joint_position_global_[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	joint_position_global_[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);


	joint_position_local_[0] = joint_position_global_[0];

	for (int i = 1; i < dof; i++)
		joint_position_local_[i] = joint_position_global_[i] - joint_position_global_[i - 1];


// Only manipulator 
	com_position_[0] = Vector3d(0.0, -0.0346, 0.2575);
	com_position_[1] = Vector3d(0.0, 0.0344, 0.4094);
	com_position_[2] = Vector3d(0.0334, 0.0265, 0.6076);
	com_position_[3] = Vector3d(0.0331, -0.0265, 0.6914);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);



	for (int i = 0; i < dof; i++)
		com_position_[i] -= joint_position_global_[i];



	for (int i = 0; i < dof; i++) {
		body_[i] = Body(mass_[i], com_position_[i], inertia_[i]);
		joint_[i] = Joint(JointTypeRevolute, axis_[i]);

		if (i == 0)
			body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
		else
			body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
	}


	////////////////////////////////////////////////////////////////

	// for (int i = 0; i < dof; i++) {
	// 	mass_[i] = 1.0;
	// 	inertia_[i] = Vector3d(0.001, 0.001, 0.001);
	// }

	// axis_[0] = Eigen::Vector3d::UnitZ();
	// axis_[1] = Eigen::Vector3d::UnitY();
	// axis_[2] = Eigen::Vector3d::UnitZ();
	// axis_[3] = -1.0*Eigen::Vector3d::UnitY();
	// axis_[4] = Eigen::Vector3d::UnitZ();
	// axis_[5] = -1.0*Eigen::Vector3d::UnitY();
	// axis_[6] = -Eigen::Vector3d::UnitZ();

	// joint_position_global_[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	// joint_position_global_[1] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	// joint_position_global_[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	// joint_position_global_[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	// joint_position_global_[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	// joint_position_global_[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	// joint_position_global_[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

	// joint_position_local_[0] = joint_position_global_[0];

	// for (int i = 1; i < dof; i++)
	// 	joint_position_local_[i] = joint_position_global_[i] - joint_position_global_[i - 1];

	// com_position_[0] = Vector3d(0.0, -0.0346, 0.2575);
	// com_position_[1] = Vector3d(0.0, 0.0344, 0.4094);
	// com_position_[2] = Vector3d(0.0334, 0.0265, 0.6076);
	// com_position_[3] = Vector3d(0.0331, -0.0265, 0.6914);
	// com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	// com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
	// com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);

	// for (int i = 0; i < dof; i++)
	// 	com_position_[i] -= joint_position_global_[i];

	// for (int i = 0; i < dof; i++) {
	// 	body_[i] = Body(mass_[i], com_position_[i], inertia_[i]);
	// 	joint_[i] = Joint(JointTypeRevolute, axis_[i]);

	// 	if (i == 0)
	// 		body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
	// 	else
	// 		body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_position_local_[i]), joint_[i], body_[i]);
	// }





}
void RobotModel::Jacobian(const int & frame_id, Vector3d& tip_pos) { //?
	MatrixXd J_temp(6, dof);
	J_temp.setZero();

    //tip_pos = com_position_[frame_id - 1];

	CalcPointJacobian6D(*model_, q_rbdl_, body_id_[frame_id - 1], tip_pos, J_temp, true);
	
	m_J_ = J_temp;
	J_temp = m_J_;
	for (int i = 0; i < 2; i++) {
		m_J_.block(i * 3, 0, 3, dof) = J_temp.block(3 - i * 3, 0, 3, dof);
	}
}
void RobotModel::Position(const int & frame_id, Vector3d& tip_pos) { // for mobile
	m_pos_ = CalcBodyToBaseCoordinates(*model_, q_rbdl_, body_id_[frame_id - 1], tip_pos, true);
}
void RobotModel::Orientation(const int & frame_id) { // for mobile
	m_Ori_ = CalcBodyWorldOrientation(*model_, q_rbdl_, body_id_[frame_id - 1], true).transpose();
}
void RobotModel::Transformation(const int & frame_id, Vector3d& tip_pos) { // for mobile
	Position(frame_id, tip_pos);
	Orientation(frame_id);
	m_Trans_.linear() = m_Ori_;
	m_Trans_.translation() = m_pos_;
}
void RobotModel::getUpdateKinematics(const VectorXd & q, const VectorXd & qdot) { // for mobile
	q_rbdl_ = q;
	qdot_rbdl_ = qdot;	
	qddot_rbdl_.setZero();
	UpdateKinematics(*model_, q_rbdl_, qdot_rbdl_, qddot_rbdl_);

}

