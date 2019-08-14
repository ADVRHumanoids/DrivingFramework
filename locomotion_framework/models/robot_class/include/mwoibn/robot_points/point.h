#ifndef __MWOIBN__ROBOT_POINTS__POINT_H
#define __MWOIBN__ROBOT_POINTS__POINT_H

#include "mwoibn/robot_class/robot_class.h"
#include "mwoibn/robot_points/rotation.h"

namespace mwoibn
{
namespace robot_points
{

class Rotation;

class Point
{

public:
  Point(unsigned int state, unsigned int dofs){
    _jacobian.setZero(state, dofs);
    _point.setZero(state);
  }

  Point(const Point& other): _jacobian(other._jacobian), _point(other._point){
  }

  Point( Point&& other): _jacobian(other._jacobian), _point(other._point){
  }

  virtual void resize(unsigned int state, unsigned int dofs){
    _jacobian.setZero(state, dofs);
    _point.setZero(state);
  }

  virtual ~Point() {}

  virtual void compute() = 0;

  virtual void computeJacobian() = 0;

  virtual void update(bool jacobian = true);

  const mwoibn::Matrix& getJacobian() const {return _jacobian;}

  const mwoibn::VectorN& get() const {return _point;}

  int size(){return _point.size();}

  int rows() {return _jacobian.rows();}
  int cols() {return _jacobian.cols();}

  virtual Point& operator=(const Point& other);

  Point& operator+(const Point& other);

  Point& operator-(const Point& other);

  Point& operator+=(const Point& other);

  Point& operator-=(const Point& other);

  void multiplyJacobian(double factor);

  void rotateFrom(mwoibn::robot_points::Rotation& rotation, bool jacobian = false);

  void rotateTo(mwoibn::robot_points::Rotation& rotation, bool jacobian = false);

protected:

  mwoibn::Matrix _jacobian;
  mwoibn::VectorN _point;

};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
