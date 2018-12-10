#ifndef __MWOIBN__ROBOT_POINTS__ROTATION_H
#define __MWOIBN__ROBOT_POINTS__ROTATION_H

#include "mwoibn/robot_class/robot_class.h"
#include "mwoibn/robot_points/point.h"

namespace mwoibn
{
namespace robot_points
{

class Rotation
{

public:
  Rotation(): rotation(_rotation){}

  virtual ~Rotation() {}

  virtual void compute() = 0;

  const mwoibn::Matrix3& rotation;

protected:
  mwoibn::Matrix3 _rotation;


};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
