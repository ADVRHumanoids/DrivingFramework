#ifndef __MWOIBN__DYNAMIC_POINTS__DYNAMIC_POINT_H
#define __MWOIBN__DYNAMIC_POINTS__DYNAMIC_POINT_H

#include "mwoibn/robot_points/state.h"

namespace mwoibn
{

namespace dynamic_points
{

class DynamicPoint: public robot_points::State
{

public:
  DynamicPoint(RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state): robot_points::State(model, state){
    // _dot_jacobian.setZero(rows(), cols());
    _constant.setZero(size());
  }

  DynamicPoint(const DynamicPoint& other):  robot_points::State(other), _constant(other._constant){
  }

  DynamicPoint( DynamicPoint&& other): robot_points::State(other), _constant(other._constant){
  }

  // virtual const mwoibn::Matrix& getJacobianDot() {return _dot_jacobian;}
  virtual const mwoibn::VectorN& getConstant() { return _constant;}

protected:
  // mwoibn::Matrix _dot_jacobian;
  mwoibn::VectorN _constant;

};

} // namespace package
} // namespace library

#endif
