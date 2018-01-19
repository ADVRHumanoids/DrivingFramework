#ifndef HIERARCHICAL_CONTROL_STEERING_ANGLE_TASK_H
#define HIERARCHICAL_CONTROL_STEERING_ANGLE_TASK_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/point_handling/robot_points_handler.h"
#include <rbdl/rbdl.h>
//#include <mgnss/controllers/steering.h>

namespace mwoibn
{
namespace hierarchical_control
{

/**
 * @brief The CartesianWorldTask class Provides the inverse kinematics task
 *to control the position of a point defined in one of a robot reference frames
 *
 */

class SteeringAngle
{
public:
  SteeringAngle(mwoibn::robot_class::Robot& robot,
              mwoibn::point_handling::Point point, mwoibn::Axis x,
              mwoibn::Axis y, mwoibn::Axis z, mwoibn::Axis axis)
      : _x_body(x), _y_body(y), _z_body(z), _axis(axis), _point(point), _robot(robot)
  {
    _x_world << 1,0,0;
    _y_world << 0,1,0;
    _z_world << 0,0,1;

  } // for now just support major axes

  ~SteeringAngle() {}

  void update(bool print = false)
  {
    _v1 = _x_world;

    _n = _z_world;

    _y = _point.getRotationWorld(_robot.state.get()).transpose() * _axis;
//    std::cout << "_y\t" << _y.transpose() << "\t";

    _v2 = _y.cross(_z_world);
    double b = 1/_v2.norm();

    _v2.normalize();
//    std::cout << "_v2\t" << _v2.transpose() << "\t";
//    std::cout << "b\t" << b << "\t";

    double cross = (_v1.cross(_v2)).transpose() * _n;
    double dot = _v1.transpose() * _v2;
    _steering = std::atan2(cross, dot);
//    mwoibn::eigen_utils::wrapToPi(_steering);

//    std::cout << _steering*180/mwoibn::PI << "\t";

// DERIVATIVE

    double O = cross*cross + dot * dot;

    double P = dot / O;
    O = -cross / O;

    mwoibn::Matrix c = _skew(_y*_n.transpose()*_y) + _y*_n.transpose()*_skew(_y);  // _z_body or world?
    mwoibn::Matrix J = -0.5*b*b*b * _n.transpose()*c;

    mwoibn::Matrix K = _skew(_y)* _n*J + b*_skew(_n)*_skew(_y);

    mwoibn::Matrix R = _v1.transpose() * K;
    mwoibn::Matrix S = _n.transpose()*_skew(_v1)*K;

    mwoibn::Matrix3 Rot;
    Rot.col(0) = _x_body;
    Rot.col(1) = _y_body;
    Rot.col(2) = _z_body;

    _J = (O*R + P*S)  * _point.getOrientationJacobian(_robot.state.get());

    if(print){
      std::cout << std::fixed << "O*R + P*S\t" << (O*R + P*S).norm() << ",\t";
      std::cout << std::fixed << "|P*S|\t" << (P*S).norm() << ",\t";
      std::cout << std::fixed << "|O*R|\t" << (O*R).norm() << ",\t";
      std::cout << std::fixed << "P*S\t" << P*S << ",\t";
      std::cout << std::fixed << "O*R\t" << O*R << ",\t";
      std::cout << "0\t" << O << ",\t";
      std::cout << "P\t" << P << ",\t";
      std::cout << "|R|\t" << R.norm() << ",\t";
      std::cout << "|S|\t" << S.norm() << ",\t";
      std::cout << "R\t" << R << ",\t";
      std::cout << "S\t" << S << ",\t";
      std::cout << "th\t" << _steering  << "\n";
    }
  }

  double get(){
    return _steering;}

  const mwoibn::Matrix& getJacobian(){return _J;}

protected:
  mwoibn::Axis _v2, _n, _v1, _y;
  mwoibn::Axis _x_body, _y_body, _z_body, _axis;

  mwoibn::Axis _x_world, _y_world,
      _z_world; // for now assume the basic initialization
  double _steering, _d_steering;
  mwoibn::point_handling::Point _point;
  mwoibn::robot_class::Robot& _robot;

  mwoibn::Matrix _J;
  mwoibn::Matrix3 _skew(mwoibn::Vector3 vec){

    mwoibn::Matrix3 mat;
    mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;

    return mat;
  }
};

 class SteeringAngleTask : public ControllerTask
 {

 public:
  SteeringAngleTask(std::vector<hierarchical_control::SteeringAngle> angels, mwoibn::robot_class::Robot& robot)
      : ControllerTask(), _robot(robot), _angels(angels)
  {
    _init(_angels.size(), _robot.getDofs());

    _ref.setZero(_angels.size());
  }

  virtual ~SteeringAngleTask() {}

  virtual void updateError()
  {
    _last_error.noalias() = _error;


    for(int i = 0; i < _angels.size(); i++){
       _angels[i].update(false);

      _error[i] = _ref[i] - _angels[i].get();
    }

      _error = eigen_utils::limitToHalfPi(_error); // make a bigger limit to avoid chattering

  }

  mwoibn::VectorN getCurrent(){
    mwoibn::VectorN current;
    current.setZero(4);

    for(int i = 0; i < _angels.size(); i++)
      current[i] = _angels[i].get();

    return current;
  }

  virtual void updateJacobian() {
    _last_jacobian.noalias() = _jacobian;

    for(int i = 0; i < _angels.size(); i++){
      _jacobian.row(i) = -_angels[i].getJacobian();
    }
  }

  virtual const mwoibn::VectorN& getReference() const { return _ref; }

  virtual void setReference(const mwoibn::VectorN& reference)
  {
    _ref = reference;
  }

  virtual double getReference(int i) const { return _ref[i]; }
  virtual void setReference(int i, double reference) { _ref[i] = reference; }

  int size(){return _angels.size();}
 protected:
  mwoibn::robot_class::Robot& _robot;
  mwoibn::VectorN _ref, _current;
  std::vector<hierarchical_control::SteeringAngle> _angels;
 };
} // namespace package
} // namespace library
#endif
