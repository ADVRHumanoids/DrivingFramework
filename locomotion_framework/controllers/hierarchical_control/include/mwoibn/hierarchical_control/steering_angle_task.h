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
      : _x_body(x), _y_body(y), _z_body(z), _axis(axis), _point(point),
        _robot(robot)
  {
    _x_world << 1, 0, 0;
    _y_world << 0, 1, 0;
    _z_world << 0, 0, 1;
    _J.setZero(1, robot.getDofs());

  } // for now just support major axes

  ~SteeringAngle() {}

  void update(bool print = false)
  {
    _v1 = _x_world;

    _n = _z_world;

    _y = _point.getRotationWorld(_robot.state.get()).transpose() * _axis;

    _v2 = _y.cross(_z_world);

    mwoibn::Scalar b = 1 / _v2.norm();

    _v2.normalize();

    mwoibn::Scalar cross = (_v1.cross(_v2)).transpose() * _n;
    mwoibn::Scalar dot = _v1.transpose() * _v2;

    _steering = std::atan2(cross, dot);

    // DERIVATIVE

    mwoibn::Scalar O = cross * cross + dot * dot;

    mwoibn::Scalar P = dot / O;
    O = -cross / O;

    mwoibn::eigen_utils::skew(_y, _s_y);
    mwoibn::eigen_utils::skew(_n, _s_n);
    mwoibn::eigen_utils::skew(_v1, _s_v1);

    _mB = _y * _n.transpose();
    _vA = _mB * _y;

    mwoibn::eigen_utils::skew(_vA, _mA);

    _mA += _mB * _s_y; // _z_body or world?
    _tA = -0.5 * b * b * b * _n.transpose() * _mA;

    _mB = _s_y * _n * _tA;
    _mB += b * _s_n * _s_y;

    _tA = _v1.transpose() * _mB;
    _tB = _n.transpose() * _s_v1 * _mB;

    _tA = O * _tA;
    _tA += P * _tB;

    _J.noalias() = _tA * _point.getOrientationJacobian(_robot.state.get());
  }

  mwoibn::Scalar get() { return _steering; }

  const mwoibn::Matrix& getJacobian() { return _J; }

protected:
  mwoibn::Axis _v2, _n, _v1, _y;
  mwoibn::Axis _x_body, _y_body, _z_body, _axis;

  mwoibn::Axis _x_world, _y_world,
      _z_world; // for now assume the basic initialization
  mwoibn::Scalar _steering, _d_steering;
  mwoibn::point_handling::Point _point;
  mwoibn::robot_class::Robot& _robot;

  mwoibn::Vector3 _vA;
  mwoibn::Vector3T _tA, _tB;

  mwoibn::Matrix3 _mA, _mB, _s_y, _s_n, _s_v1;

  mwoibn::Matrix _J;
};

class SteeringAngleTask : public ControllerTask
{

public:
  SteeringAngleTask(std::vector<hierarchical_control::SteeringAngle> angels,
                    mwoibn::robot_class::Robot& robot)
      : ControllerTask(), _robot(robot), _angels(angels)
  {
    _init(_angels.size(), _robot.getDofs());

    _ref.setZero(_angels.size());
    _current.setZero(_angels.size());
    _resteer.setConstant(_angels.size(), false);
  }

  virtual ~SteeringAngleTask() {}

  virtual void updateError()
  {
    _last_error.noalias() = _error;

    for (int i = 0; i < _angels.size(); i++)
    {
      _resteer[i] = false;
      _angels[i].update(false);

      mwoibn::eigen_utils::wrapToPi(_ref[i]);
      _error[i] = _ref[i] - _angels[i].get();

      _limit2PI(i);

      mwoibn::eigen_utils::wrapToPi(_error[i]);
      mwoibn::eigen_utils::wrapToPi(_ref[i]);

      if ( _error[i] > 0 &&  std::fabs(_error[i] - mwoibn::PI) < 50*mwoibn::PI/180){
		
        std::cout << i << "\t" << _ref[i] <<  "\t" << _error[i] << std::endl;
//        if(!_resteer[i]){
//        std::cout << i << "\t change config" << std::endl;
        _error[i]-= mwoibn::PI;
//        _ref[i] -= mwoibn::PI;
        _resteer[i] = true;
//        }
//        std::cout << "\t" << _ref[i] <<  "\t" << _error[i] << std::endl;

      }

      else if (_error[i] < 0 &&  std::fabs(_error[i] + mwoibn::PI) < 50*mwoibn::PI/180){
        std::cout << i << "\t" << _ref[i] <<  "\t" << _error[i] << std::endl;
//        if(!_resteer[i]){

//        std::cout << i << "\t change config" << std::endl;
        _error[i]+= mwoibn::PI;
//        _ref[i] += mwoibn::PI;
        _resteer[i] = true;
//        std::cout << "\t" << _ref[i] <<  "\t" << _error[i] << std::endl;
//        }
      }
//      else
//        _resteer[i] = false;

      if (_error[i] > 30*mwoibn::PI/180)
        _error[i] = 30*mwoibn::PI/180;
      else if (_error[i] < -30*mwoibn::PI/180)
        _error[i] = -30*mwoibn::PI/180;
//      while(_error[i] < -mwoibn::PI){
//        std::cout << i << "\t" << _ref[i] <<  "\t" << _error[i] << std::endl;

//        _error[i]+= mwoibn::PI;
//        _ref[i] += mwoibn::PI;
//        std::cout << "\t" << _ref[i] <<  "\t" << _error[i] << std::endl;

//      }




    }
  }

  const mwoibn::VectorN& getCurrent()
  {

    for (int i = 0; i < _angels.size(); i++)
      _current[i] = _angels[i].get();

    return _current;
  }

  virtual void updateJacobian()
  {
    _last_jacobian.noalias() = _jacobian;

    for (int i = 0; i < _angels.size(); i++)
    {
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

  int size() { return _angels.size(); }

  bool resteer(int i){return _resteer[i];}
  const mwoibn::VectorBool& resteer(){return _resteer;}
protected:
  mwoibn::robot_class::Robot& _robot;
  mwoibn::VectorN _ref, _current;
  std::vector<hierarchical_control::SteeringAngle> _angels;
  mwoibn::VectorBool _resteer;

  void _limit2PI(int i){

    if(_error[i] - _last_error[i] > mwoibn::PI){
      if(i == 0)
//      std::cout << i << "\t" << _error[i] << "\t" << _last_error[i] << "\t" << _ref[i];

        _ref[i] -= mwoibn::TWO_PI;
        _error[i] = _ref[i] - _angels[i].get();
      if(i == 9)
//        std::cout << "\t" << _ref[i] <<  "\t" << _error[i] << std::endl;
        _limit2PI(i);
      }
    else if (_last_error[i] - _error[i] > mwoibn::PI){
      if (i == 0)
//      std::cout << i << "\t" << _error[i] << "\t" << _last_error[i] << "\t" << _ref[i];
      _ref[i] += mwoibn::TWO_PI;
      _error[i] = _ref[i] - _angels[i].get();
      if(i == 0)
//        std::cout << "\t" << _ref[i] << "\t" << _error[i] << std::endl;
      _limit2PI(i);
    }
  }
};
} // namespace package
} // namespace library
#endif
