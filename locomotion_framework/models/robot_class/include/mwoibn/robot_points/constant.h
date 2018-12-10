#ifndef __MWOIBN__ROBOT_CLASS__CONSTANT_H
#define __MWOIBN__ROBOT_CLASS__CONSTANT_H

#include "mwoibn/robot_points/point.h"

namespace mwoibn
{
namespace robot_points
{

class Constant: public Point
{

public:
  Constant(mwoibn::VectorN point, mwoibn::Matrix jacobian): Point(point.size(), jacobian.cols()){
  }

  using Point::Point;


  virtual ~Constant() {}

  virtual void compute(){}

  virtual void computeJacobian(){}


};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
