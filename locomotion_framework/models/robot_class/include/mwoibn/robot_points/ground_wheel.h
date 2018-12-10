#ifndef __MWOIBN__ROBOT_POINTS__GROUND_WHEEL_H
#define __MWOIBN__ROBOT_POINTS__GROUND_WHEEL_H

#include "mwoibn/robot_points/rotation.h"

namespace mwoibn
{
namespace robot_points
{

class GroundWheel: public Rotation
{

public:
  GroundWheel(const mwoibn::Vector3& axis_world, const mwoibn::Vector3& ground_normal): _axis(axis_world), _ground_normal(ground_normal) {
      compute();
  }

  virtual ~GroundWheel() {}

  virtual void compute(){
        _rotation.col(0) = _axis.cross(_ground_normal).normalized();
        _rotation.col(1) = _ground_normal.cross(_rotation.col(0));
        _rotation.col(2) = _ground_normal;
  }

protected:
  const mwoibn::Vector3& _axis;
  const mwoibn::Vector3& _ground_normal;



};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
