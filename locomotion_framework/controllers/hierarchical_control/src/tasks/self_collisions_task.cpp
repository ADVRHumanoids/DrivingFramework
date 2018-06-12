#include "mwoibn/hierarchical_control/tasks/self_collisions_task.h"

//namespace RbdlMath = RigidBodyDynamics::Math;

void mwoibn::hierarchical_control::tasks::SelfCollisions::updateError(){
	_last_error = _error;
  _robot.updateCollisions();
  mwoibn::VectorN distances = _robot.getDistances();
  _error = mwoibn::VectorN::Zero(_robot.getPairsNumber());
  for (int i = 0; i < _robot.getPairsNumber(); i++){
    if (fabs(distances[i])-_safety_limit[i] < 0  && distances[i] > 0){
			_error[i] = (distances[i]-_safety_limit[i])*(distances[i]-_safety_limit[i]);
    }
	}
}

void mwoibn::hierarchical_control::tasks::SelfCollisions::updateJacobian(){

	_last_jacobian = _jacobian;

  mwoibn::VectorN distances = _robot.getDistances();

  mwoibn::Matrix temp_jacobian_1 = mwoibn::Matrix::Zero(3,_robot.getRobotDofs());
  mwoibn::Matrix temp_jacobian_2 = mwoibn::Matrix::Zero(3,_robot.getRobotDofs());
  mwoibn::Vector3 unit_vector;
  _jacobian = mwoibn::Matrix::Zero(_robot.getPairsNumber(),_robot.getRobotDofs());

  for (int i = 0; i < _robot.getPairsNumber(); i++){

    if (distances[i]-_safety_limit[i] <= 0 && distances[i] > 0){
      _robot.updatePointHendler(i);
			distances[i] = 2*(distances[i] - _safety_limit[i]);
      temp_jacobian_1 = _robot.pair_ik_ptr->getPointJacobian(2 * i);
      temp_jacobian_2 = _robot.pair_ik_ptr->getPointJacobian(2 * i + 1);
			temp_jacobian_2 -= temp_jacobian_1;
      unit_vector = _robot.pair_ik_ptr->getPointStateWorld(2*i+1) - _robot.pair_ik_ptr->getPointStateWorld(2*i);
			unit_vector.normalize();

			_jacobian.row(i) = distances[i]*unit_vector.transpose()*temp_jacobian_2;
		}
	}
}
