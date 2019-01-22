#include "mwoibn/robot_points/point.h"

void mwoibn::robot_points::Point::update(bool jacobian) {
    compute();
    if(jacobian)
      computeJacobian();
  }

mwoibn::robot_points::Point& mwoibn::robot_points::Point::operator=(const mwoibn::robot_points::Point& other) {
    if (this != &other){
        _jacobian = other._jacobian;
        _point = other._point;
    }
    return *this;
}

mwoibn::robot_points::Point& mwoibn::robot_points::Point::operator+(const mwoibn::robot_points::Point& other) {
    _jacobian += other._jacobian;
    _point += other._point;
return *this;
}

mwoibn::robot_points::Point& mwoibn::robot_points::Point::operator-(const mwoibn::robot_points::Point& other) {
    _jacobian -= other._jacobian;
    _point -= other._point;
return *this;
}

mwoibn::robot_points::Point& mwoibn::robot_points::Point::operator+=(const mwoibn::robot_points::Point& other){
    _jacobian += other._jacobian;
    _point += other._point;
    return *this;
}

mwoibn::robot_points::Point& mwoibn::robot_points::Point::operator-=(const mwoibn::robot_points::Point& other){
    _jacobian -= other._jacobian;
    _point -= other._point;
    return *this;
}

void mwoibn::robot_points::Point::multiplyJacobian(double factor){
  _jacobian = factor*_jacobian;
}

void mwoibn::robot_points::Point::rotateFrom(mwoibn::robot_points::Rotation& rotation, bool jacobian){
    _point = rotation.reverse(*this);

    if(jacobian)
      throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__ + std::string(" Jacobian rotation"));
}

void mwoibn::robot_points::Point::rotateTo(mwoibn::robot_points::Rotation& rotation, bool jacobian){
  _point = rotation.direct(*this);
  if(jacobian)
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__ + std::string(" Jacobian rotation"));

}
