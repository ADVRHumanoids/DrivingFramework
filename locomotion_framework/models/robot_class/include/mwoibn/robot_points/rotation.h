#ifndef __MWOIBN__ROBOT_POINTS__ROTATION_H
#define __MWOIBN__ROBOT_POINTS__ROTATION_H

#include "mwoibn/robot_class/robot_class.h"
#include "mwoibn/robot_points/point.h"

namespace mwoibn
{
namespace robot_points
{

class Point;

class Rotation
{

public:
  Rotation(): rotation(_rotation){
      _rotation.setIdentity();
  }

  virtual ~Rotation() {}

  virtual void compute() = 0;

  // if this would be a transformation it could be better to separate computation and get to have different return types?
  mwoibn::Vector3& direct(const mwoibn::robot_points::Point& point);

  const mwoibn::Vector3& reverse(const mwoibn::robot_points::Point& point);

  const mwoibn::Matrix3& rotation;

protected:
  mwoibn::Matrix3 _rotation;
  mwoibn::Vector3 _last_point;


};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
