#include "mwoibn/robot_points/rotation.h"

  mwoibn::Vector3& mwoibn::robot_points::Rotation::direct(const mwoibn::robot_points::Point& point) {
      _last_point.noalias() = _rotation*point.get();
      return _last_point;
  }

  const mwoibn::Vector3& mwoibn::robot_points::Rotation::reverse(const mwoibn::robot_points::Point& point){
    _last_point.noalias() = _rotation.transpose()*point.get();
    return _last_point;
  }
