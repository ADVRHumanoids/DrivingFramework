#ifndef __MWOIBN__DYNAMIC_POINTS__FORCE_BASE_H
#define __MWOIBN__DYNAMIC_POINTS__FORCE_BASE_H

#include "mwoibn/dynamic_points/dynamic_point.h"
//#include "mwoibn/point_handling/frame_plus.h"
// #include "mwoibn/point_handling/linear_velocity.h"
//#include "mwoibn/point_handling/linear_acceleration.h"
#include "mwoibn/dynamic_models/basic_model.h"

namespace mwoibn
{

namespace dynamic_points
{

class ForceBase: public DynamicPoint
{

public:


  ForceBase(RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state, mwoibn::dynamic_models::BasicModel& dynamic_model, mwoibn::dynamic_points::DynamicPoint& frame, mwoibn::Interface interface = "OVERALL_FORCE"):
    DynamicPoint(model, state), _frame(frame), _dynamic_model(dynamic_model), _interface(interface){
          _init();
  }

  ForceBase(mwoibn::robot_class::Robot& robot, mwoibn::dynamic_models::BasicModel& dynamic_model, mwoibn::dynamic_points::DynamicPoint& frame, mwoibn::Interface interface = "OVERALL_FORCE"):
      DynamicPoint(robot.getModel(), robot.state), _frame(frame), _dynamic_model(dynamic_model), _interface(interface){
    _init();
  }


  ForceBase( ForceBase&& other, dynamic_points::DynamicPoint& frame)
      : DynamicPoint(other), _frame(frame), _dynamic_model(other._dynamic_model), _interface(other._interface)
  {
    _init();
  }

  ForceBase(const ForceBase& other, dynamic_points::DynamicPoint& frame)
      : DynamicPoint(other),  _frame(frame), _dynamic_model(other._dynamic_model), _interface(other._interface)
  {
    _init();
  }

  ForceBase( ForceBase&& other)
      : DynamicPoint(other),  _frame(other._frame), _dynamic_model(other._dynamic_model), _interface(other._interface)
  {
    _init();
  }

  ForceBase(const ForceBase& other)
      : DynamicPoint(other),  _frame(other._frame), _dynamic_model(other._dynamic_model), _interface(other._interface)
  {
    _init();
  }

  virtual ~ForceBase() {
    _dynamic_model.unsubscribe(mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE);

  }

  virtual void update(bool jacobian) {
      compute();
      if(jacobian)
        computeJacobian();
    }

  using DynamicPoint::operator=;

  void computeJacobian(){
    _jacobian = _contacts_inverse->get();
  }

  void compute(){

    //_inertia_inverse->compute(_dynamic_model.getInertia());
    _point_jacobian = _frame.getJacobian().block<6,6>(0,0);
    _point_transposed = _point_jacobian.transpose();

    //std::cout << "_point_jacobian\n" << _point_jacobian << std::endl;

    _point_inverse.noalias() = _point_jacobian*_dynamic_model.getInertiaInverse().block<6,6>(0,0);

    _point_temp.noalias() = _point_inverse*_point_transposed;

    _contacts_inverse->compute(_point_temp);

    _jacobian_temp.noalias() = _contacts_inverse->get()*_point_inverse;
    //std::cout << "jacobian temp\n" << _jacobian_temp << std::endl;
    _point.noalias() = _jacobian_temp*(_state[_interface].get().head<6>());
    //std::cout << "state\t" << _state[_interface].get().head<6>().transpose() << std::endl;
    _point += _contacts_inverse->get()*_frame.getConstant();

  }
protected:
  //mwoibn::point_handling:: _velocity;
  std::unique_ptr<mwoibn::Inverse> _contacts_inverse;
  mwoibn::dynamic_models::BasicModel& _dynamic_model;
  //mwoibn::Matrix _temp;
  mwoibn::Matrix _point_jacobian, _jacobian_temp, _point_transposed, _point_inverse, _point_temp;
  //mwoibn::Matrix3 _inverse;
  mwoibn::dynamic_points::DynamicPoint& _frame;
  mwoibn::Interface _interface;
  void _init(){
    resize(_frame.rows(), _frame.cols());
    _dynamic_model.subscribe(mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE);

    //_inertia_inverse.reset(new mwoibn::PseudoInverse(mwoibn::Matrix::Zero(_state.acceleration.size(), _state.acceleration.size())));
    _contacts_inverse.reset(new mwoibn::Inverse(mwoibn::Matrix::Zero(_frame.size(), _frame.size())));
    _jacobian.setZero(_frame.size(), _frame.size());
    _jacobian_temp.setZero(_frame.size(), 6);

    _point_jacobian.setZero(_frame.rows(), 6);
    _point_transposed.setZero(6, _frame.rows());
    _point_inverse.setZero(_frame.rows(), 6);
    _point_temp.setZero(_frame.rows(), _frame.rows());

    _point.setZero(_frame.size());
  }
};

} // namespace package
} // namespace library

#endif
