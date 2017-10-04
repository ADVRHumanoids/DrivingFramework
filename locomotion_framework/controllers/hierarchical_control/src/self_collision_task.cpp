#include "mwoibn/hierarchical_control/self_collision_task.h"

//namespace RbdlMath = RigidBodyDynamics::Math;

void mwoibn::hierarchical_control::SelfCollisionTask::updateError()
{

  _last_error = _error;
  _robot.updateCollisions();
  mwoibn::VectorN distances = _robot.getDistances();

  //	int collisions = 0;
  for (int i = 0; i < _robot.getPairsNumber(); i++)
  {
    //		if (distances[i] <= _safety_limit[i] && distances[i] > 0)
    //collisions++;
    // check if pair is close to collision, and compute a cost function, second
    // condition is to remove cases when collision already occured. It is needed
    // as the SCH works without a depth penetration algorthim, and therefore in
    // case of collision returnes zero witness points
    distances[i] = (distances[i] - _safety_limit[i] <= 0 && distances[i] > 0)
                       ? (distances[i] - _safety_limit[i]) *
                             (distances[i] - _safety_limit[i])
                       : 0;
  }
  _error[0] = distances.sum(); //???
}

void mwoibn::hierarchical_control::SelfCollisionTask::updateJacobian()
{

  _last_jacobian = _jacobian;

  mwoibn::VectorN distances = _robot.getDistances();

  mwoibn::Matrix temp_jacobian_1 =
      mwoibn::Matrix::Zero(3, _robot.getRobotDofs());
  mwoibn::Matrix temp_jacobian_2 =
      mwoibn::Matrix::Zero(3, _robot.getRobotDofs());
  mwoibn::Vector3 unit_vector;

  _jacobian = mwoibn::Matrix::Zero(_jacobian.rows(), _jacobian.cols());

  for (int i = 0; i < _robot.getPairsNumber(); i++)
  {

    if (distances[i] - _safety_limit[i] <= 0 && distances[i] > 0)
    {
      _robot.updatePointHendler(i);
      distances[i] = 2 * (distances[i] - _safety_limit[i]);
      temp_jacobian_1 = _robot.pair_ik_ptr->getPointJacobian(2 * i);
      temp_jacobian_2 = _robot.pair_ik_ptr->getPointJacobian(2 * i + 1);
      temp_jacobian_2 -= temp_jacobian_1;
      unit_vector = _robot.pair_ik_ptr->getPointStateWorld(2 * i + 1) -
                    _robot.pair_ik_ptr->getPointStateWorld(2 * i);

      unit_vector.normalize();

      _jacobian += distances[i] * unit_vector.transpose() * temp_jacobian_2;
    }
  }
  //  for (int i = 0; i < _jacobian.size(); i++)
  //    std::cout << _jacobian(i) << ", ";
  //
  // std::cout << "\n";
}
