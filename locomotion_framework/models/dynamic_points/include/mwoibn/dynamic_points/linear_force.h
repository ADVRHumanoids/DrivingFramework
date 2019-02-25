#ifndef __MWOIBN__DYNAMIC_POINTS__LINEAR_FORCE_H
#define __MWOIBN__DYNAMIC_POINTS__LINEAR_FORCE_H

#include "mwoibn/dynamic_points/dynamic_point.h"
#include "mwoibn/point_handling/frame_plus.h"
#include "mwoibn/point_handling/linear_velocity.h"
#include "mwoibn/point_handling/linear_acceleration.h"
#include "mwoibn/dynamic_models/basic_model.h"

namespace mwoibn
{

namespace dynamic_points
{

  // Computes the point force given desired accleration
class LinearForce: public DynamicPoint
{

public:


  LinearForce(point_handling::FramePlus& frame, mwoibn::dynamic_models::BasicModel& dynamic_model, mwoibn::Vector3& des_acc):
    DynamicPoint(frame.getModel(), frame.getState()), _frame(frame), _velocity(_frame), _dynamic_model(dynamic_model), _des_acc(des_acc){
          _init();
  }

  LinearForce(const LinearForce& other, point_handling::FramePlus& frame)
      : DynamicPoint(other), _frame(frame), _velocity(_frame), _dynamic_model(other._dynamic_model), _des_acc(other._des_acc)
  {
    _init();
  }

  LinearForce( LinearForce&& other, point_handling::FramePlus& frame)
      : DynamicPoint(other), _frame(frame), _velocity(_frame), _dynamic_model(other._dynamic_model), _des_acc(other._des_acc)
  {
    _init();
  }

  LinearForce( LinearForce&& other)
      : DynamicPoint(other), _frame(other._frame), _velocity(_frame), _dynamic_model(other._dynamic_model), _des_acc(other._des_acc)
  {
    _init();
  }

  LinearForce(const LinearForce& other)
      : DynamicPoint(other), _frame(other._frame), _velocity(_frame), _dynamic_model(other._dynamic_model), _des_acc(other._des_acc)
  {
    _init();
  }

  virtual ~LinearForce() {
    _dynamic_model.unsubscribe(mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE);
  }

  using Point::operator=;

    virtual void compute();

    virtual void computeJacobian();


protected:
  //mwoibn::point_handling::LinearVelocity _velocity;
  std::unique_ptr<mwoibn::PseudoInverse> _contacts_inverse;

  mwoibn::Matrix _point_jacobian, _point_transposed, _point_inverse, _point_temp;

  point_handling::FramePlus& _frame;
  point_handling::LinearVelocity _velocity;
  //point_handling::LinearAcceleration _acceleration; // should I make a class to compute \dJ\dq somewhere? For tomorrow keep this

  mwoibn::Vector3& _des_acc;
  mwoibn::dynamic_models::BasicModel& _dynamic_model;

  //mwoibn::Interface _interface;

  void _init(){
    _dynamic_model.subscribe(mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE);

    //_inertia_inverse.reset(new mwoibn::PseudoInverse(mwoibn::Matrix::Zero(_state.acceleration.size(), _state.acceleration.size())));
    _contacts_inverse.reset(new mwoibn::PseudoInverse(mwoibn::Matrix3::Zero()));
    _jacobian.setZero(_frame.size(), _frame.size());
    _point_jacobian.setZero(_frame.size(), _frame.dofs());
    _point_transposed.setZero(_frame.dofs(), _frame.size());
    _point_inverse.setZero(_frame.size(), _frame.dofs());
    _point_temp.setZero(_frame.size(), _frame.size());

    _point.setZero(_frame.size());
  }
};

} // namespace package
} // namespace library

#endif
