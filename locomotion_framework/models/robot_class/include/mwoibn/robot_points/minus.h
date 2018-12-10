#ifndef __MWOIBN__ROBOT_CLASS__MINUS_H
#define __MWOIBN__ROBOT_CLASS__MINUS_H

#include "mwoibn/robot_points/state.h"

namespace mwoibn
{
namespace robot_points
{

class Minus: public Point
{

public:
  Minus(robot_points::Point& minuend, robot_points::Point& subtrahend): Point(minuend), _minuend(minuend), _subtrahend(subtrahend){
    if(minuend.size() != subtrahend.size() || minuend.cols() != subtrahend.cols())
      throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": received uncombatibile robot points"));
    *this = minuend - subtrahend;
  }

  virtual ~Minus() {}

  using Point::operator=;

  virtual void compute(){
    _point = _minuend.get();
    _point -= _subtrahend.get();
  }

  virtual void computeJacobian(){
    _jacobian = _minuend.getJacobian();
    _jacobian -= _subtrahend.getJacobian();
  }

  Point& _minuend;
  Point& _subtrahend;

};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
