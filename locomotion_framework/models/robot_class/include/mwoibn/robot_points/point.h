#ifndef __MWOIBN__ROBOT_CLASS__POINT_H
#define __MWOIBN__ROBOT_CLASS__POINT_H

#include "mwoibn/robot_class/robot_class.h"

namespace mwoibn
{
namespace robot_points
{

class Point
{

public:
  Point(RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state): _state(state), _model(model){
    _jacobian.setZero(3, _state.size());
  }

  virtual ~Point() {}

  virtual void compute() = 0;

  virtual void computeJacobian() = 0;

  void update(bool jacobian = true) {
      compute();
      if(jacobian)
        computeJacobian();
    }

  const mwoibn::Matrix& getJacobian() const {return _jacobian;}
  const mwoibn::VectorN& get() const {return _point;}

protected:
  RigidBodyDynamics::Model& _model;
  const mwoibn::robot_class::State& _state;
  mwoibn::Matrix _jacobian;
  mwoibn::VectorN _point;

};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
