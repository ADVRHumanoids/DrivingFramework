#ifndef __MWOIBN__DYNAMIC_POINTS__FORCE_H
#define __MWOIBN__DYNAMIC_POINTS__FORCE_H

#include "mwoibn/robot_points/state.h"
#include "mwoibn/point_handling/frame_plus.h"
#include "mwoibn/point_handling/linear_velocity.h"
//#include "mwoibn/point_handling/linear_acceleration.h"
#include "mwoibn/dynamic_models/basic_model.h"

namespace mwoibn
{

namespace dynamic_points
{

class Force: public robot_points::State
{

public:


  Force(RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state, mwoibn::dynamic_models::BasicModel& dynamic_model, mwoibn::robot_points::Point& frame, mwoibn::Interface interface = "OVERALL_FORCE"):
    robot_points::State(model, state), _frame(frame), _dynamic_model(dynamic_model), _interface(interface){
          _init();
  }

  Force(mwoibn::robot_class::Robot& robot, mwoibn::dynamic_models::BasicModel& dynamic_model, mwoibn::robot_points::Point& frame, mwoibn::Interface interface = "OVERALL_FORCE"):
      robot_points::State(robot.getModel(), robot.state), _frame(frame), _dynamic_model(dynamic_model), _interface(interface){
    _init();
  }


  Force( Force&& other, robot_points::Point& frame)
      : robot_points::State(other), _frame(frame), _dynamic_model(other._dynamic_model), _interface(other._interface)
  {
    _init();
  }

  Force(const Force& other, robot_points::Point& frame)
      : robot_points::State(other),  _frame(frame), _dynamic_model(other._dynamic_model), _interface(other._interface)
  {
    _init();
  }

  Force( Force&& other)
      : robot_points::State(other),  _frame(other._frame), _dynamic_model(other._dynamic_model), _interface(other._interface)
  {
    _init();
  }

  Force(const Force& other)
      : robot_points::State(other),  _frame(other._frame), _dynamic_model(other._dynamic_model), _interface(other._interface)
  {
    _init();
  }

  virtual ~Force() {
    _dynamic_model.unsubscribe(mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA);

  }

  using Point::operator=;

    virtual void compute();

    virtual void computeJacobian();


protected:
  //mwoibn::point_handling::LinearVelocity _velocity;
  std::unique_ptr<mwoibn::PseudoInverse> _inertia_inverse, _contacts_inverse;
  mwoibn::dynamic_models::BasicModel& _dynamic_model;
  //mwoibn::Matrix _temp;
  mwoibn::Matrix _point_jacobian, _point_transposed, _point_inverse, _point_temp;
  //mwoibn::Matrix3 _inverse;
  mwoibn::robot_points::Point& _frame;
  mwoibn::Interface _interface;
  void _init(){
    _dynamic_model.subscribe(mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA);

    _inertia_inverse.reset(new mwoibn::PseudoInverse(mwoibn::Matrix::Zero(_state.acceleration.size(), _state.acceleration.size())));
    _contacts_inverse.reset(new mwoibn::PseudoInverse(mwoibn::Matrix3::Zero()));
    _jacobian.setZero(_frame.size(), _frame.cols());
    _point_jacobian.setZero(_frame.rows(), _frame.cols());
    _point_transposed.setZero(_frame.rows(), _frame.rows());
    _point_inverse.setZero(_frame.rows(), _frame.cols());
    _point_temp.setZero(_frame.rows(), _frame.rows());

    _point.setZero(_frame.size());
  }
};

} // namespace package
} // namespace library

#endif
