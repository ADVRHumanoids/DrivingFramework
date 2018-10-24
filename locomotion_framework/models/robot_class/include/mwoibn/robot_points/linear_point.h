#ifndef __MWOIBN__ROBOT_CLASS__LINEAR_POINT_H
#define __MWOIBN__ROBOT_CLASS__LINEAR_POINT_H

#include "mwoibn/robot_class/point.h"
#include "mwoibn/robot_class/position.h"


namespace mwoibn
{
namespace robot_class
{

class LinearPoint: public Point
{

public:
  LinearPoint(RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state, mwoibn::point_handling::Position& position): Point(model, state), point(position){
    _point.setZero(3);
  }

  LinearPoint(RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state, mwoibn::point_handling::Position position): Point(model, state), point(position){
    _point.setZero(3);
  }

  virtual ~LinearPoint() {}

  void compute(){_point.noalias() = point.getPointWorld();}

  void computeJacobian() {_jacobian.noalias() = point.getPositionJacobian();}

  mwoibn::point_handling::Position point;


};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
