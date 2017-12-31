#ifndef HIERARCHICAL_CONTROL_CASTOR_ANGLE_TASK_H
#define HIERARCHICAL_CONTROL_CASTOR_ANGLE_TASK_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/point_handling/robot_points_handler.h"
#include <rbdl/rbdl.h>

namespace mwoibn
{
namespace hierarchical_control
{

/**
 * @brief The CartesianWorldTask class Provides the inverse kinematics task
 *to control the position of a point defined in one of a robot reference frames
 *
 */

class CastorAngle
{
public:
  CastorAngle(mwoibn::robot_class::Robot& robot,
              mwoibn::point_handling::Point point, mwoibn::Axis x,
              mwoibn::Axis y, mwoibn::Axis z, mwoibn::Axis axis)
      : _x_body(x), _y_body(y), _z_body(z), _point(point), _robot(robot), _axis(axis)
  {
    _x_world << 1,0,0;
    _y_world << 0,1,0;
    _z_world << 0,0,1;

  } // for now just support major axes

  ~CastorAngle() {}

  void update()
  {
    // axis - z
    _v1 = _point.getRotationWorld(_robot.state.get()).transpose() * _z_world;

//    std::cout << "_v1\t" << _v1.transpose();

    _n = _point.getRotationWorld(_robot.state.get()).transpose() *
         _axis; // this could be optimized

//    std::cout << "_n\t" << _n.transpose();

    _v2 = _z_world - _n * _z_world.transpose() * _n;

    double b = 1/_v2.norm();
    _v2.normalize();
//    std::cout << "_v2\t" << _v2.transpose();

    double cross = (_v1.cross(_v2)).transpose() * _n;
    double dot = _v1.transpose() * _v2;
    _castor = std::atan2(cross, dot);

// DERIVATIVE

    double A = cross*cross + dot * dot;

    double B = dot / A;
    A = -cross / A;

    mwoibn::Matrix C = (0.5*b*b*_z_world*_z_world.transpose() - mwoibn::Matrix::Identity(3,3) - 0.5*b*b*_n*_n.transpose()*_z_world*_z_world.transpose())*b;

    C = -C*(_skew(_n*_z_world.transpose()*_n) + _n*_z_world.transpose()*_skew(_n));

    mwoibn::Matrix E = _v1.transpose()*C - _v2.transpose()*_skew(_v1);

    mwoibn::Matrix F = _n.transpose()*_skew(_v2)*_skew(_v1) + _n.transpose()*_skew(_v1)*C - (_skew(_v1)*_v2).transpose()*_skew(_n);

    _J = (A*E + B*F) * _point.getOrientationJacobian(_robot.state.get());


  }

  double get(){
    return _castor;}
  const mwoibn::Matrix& getJacobian(){return _J;}

protected:
  mwoibn::Axis _x_body, _y_body, _z_body, _axis;
  mwoibn::Axis _v2, _n, _v1;

  mwoibn::Axis _x_world, _y_world,
      _z_world; // for now assume the basic initialization
  double _castor, _d_castor;
  mwoibn::point_handling::Point _point;
  mwoibn::robot_class::Robot& _robot;

  mwoibn::Matrix _J;
  mwoibn::Matrix3 _skew(mwoibn::Vector3 vec){

    mwoibn::Matrix3 mat;
    mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;

    return mat;
  }
};

 class CastorAngleTask : public ControllerTask
 {

 public:
  CastorAngleTask(std::vector<hierarchical_control::CastorAngle> angels, mwoibn::robot_class::Robot& robot)
      : ControllerTask(), _robot(robot), _angels(angels)
  {
    _init(_angels.size(), _robot.getDofs());
    _ref.setZero(_angels.size());
  }

  virtual ~CastorAngleTask() {}

  virtual void updateError()
  {
//    std::cout << "update" << std::endl;
    _last_error.noalias() = _error;

    for(int i = 0; i < _angels.size(); i++){
      _angels[i].update();
      _error[i] = _ref[i] - _angels[i].get();
//      std::cout << "\t" << _angels[i].get()*180/3.14;// << std::endl;
//      std::cout << "\t" << _error[i]*180/3.14;// << std::endl;
    }

//    std::cout << "\t" << _angels[1].get()*180/3.14;// << std::endl;
//    std::cout << "\t" << _error[1]*180/3.14;
    _error = eigen_utils::limitToHalfPi(_error); // make a bigger limit to avoid chattering
//    std::cout << "\t" << _error[1]*180/3.14;// << std::endl;

//    std::cout << std::fixed << "error\t" << _error.transpose() * 180 / 3.14 << "\n";
//    std::cout << std::endl;
  }

  virtual void updateJacobian() {
    _last_jacobian.noalias() = _jacobian;

    for(int i = 0; i < _angels.size(); i++){
      _jacobian.row(i) = -_angels[i].getJacobian();
    }
  }

  mwoibn::VectorN getCurrent(){
    mwoibn::VectorN current;
    current.setZero(4);

    for(int i = 0; i < _angels.size(); i++)
      current[i] = _angels[i].get();

    return current;
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
  std::vector<hierarchical_control::CastorAngle> _angels;
 };
} // namespace package
} // namespace library
#endif
