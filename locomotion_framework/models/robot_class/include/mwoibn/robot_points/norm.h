#ifndef __MWOIBN__ROBOT_CLASS__NORM_H
#define __MWOIBN__ROBOT_CLASS__NORM_H

#include "mwoibn/robot_points/point.h"

namespace mwoibn
{
namespace robot_points
{

class Norm: public Point
{

public:
  Norm(robot_points::Point& original): Point(1, original.cols()), _original(original){
  }

  virtual ~Norm() {}

  using Point::operator=;

  virtual void compute(){
    _point = _original.get().transpose()*_original.get();
  }

  virtual void computeJacobian(){
    _jacobian.noalias() = 2*_original.get().transpose()*_original.getJacobian();
  }

  Point& _original;

};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
