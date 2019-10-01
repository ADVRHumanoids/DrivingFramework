#include "mwoibn/robot_points/point_contact.h"

void mwoibn::robot_points::PointContact::_initDirections(
    mwoibn::Matrix6 directions)
{

  for (int i = 3; i < directions.cols(); i++)
  {
    std::cout << i << std::endl;
    if (directions.block<3, 1>(3, i).norm() > 0.0001)
      _directions.block<3, 1>(0, i-3) = directions.block<3, 1>(3, i).normalized();
    else
      _directions.block<3, 1>(0, i-3) << 0, 0, 0;
  }

  _directions = _directions.transpose().eval();

//  _directions.transposeInPlace();
}


const mwoibn::Matrix&
mwoibn::robot_points::PointContact::getPointJacobian(mwoibn::Matrix3 rotation_matrix)
{

  _jacobian.setZero();

  if (!_is_active) return _jacobian;

  _transformation.noalias() = _directions * rotation_matrix;
  _jacobian.noalias() = _transformation * _frame.getPositionJacobian();

  return _jacobian;
}

const mwoibn::VectorN& mwoibn::robot_points::PointContact::getPosition()
{
  return _frame.getLinearWorld();
}
