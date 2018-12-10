#ifndef __MWOIBN__ROBOT_CLASS__JOINT_H
#define __MWOIBN__ROBOT_CLASS__JOINT_H

#include "mwoibn/robot_points/state.h"
#include "mwoibn/point_handling/frame_plus.h"

namespace mwoibn
{
namespace robot_points
{

/* supports rotational and prismatic joints */
class Joint: public State
{

public:
  template<typename Body>
  Joint(Body body, RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state): State(model, state),
        _frame(body, model, state), _velocity(_frame){
          _init();
  }

  template<typename Body>
  Joint(Body body, mwoibn::robot_class::Robot& robot): State(robot.getModel(), robot.state), _frame(body, _model, _state), _velocity(_frame) {
    _init();
  }
  using Point::operator=;

  virtual ~Joint() {}

  virtual void compute(){
    _axis_world = _frame.orientation.getWorld().rotate(_axis);
    _torque = _axis_world*_state.torque.get(_dof);
    _point = _frame.position.getWorld();

  }

  virtual void computeJacobian(){
    _jacobian.noalias() = _velocity.getJacobian();
  }

  const mwoibn::Vector3& torque(){return _torque;}
  double scalarTorque(){return _state.torque.get(_dof);}
  const mwoibn::Vector3& axis(){return _axis_world;}

  const  mwoibn::point_handling::FramePlus& frame(){return _frame;}

protected:
  mwoibn::point_handling::FramePlus _frame;
  mwoibn::Axis _axis, _axis_world;
  mwoibn::Vector3 _torque;
  mwoibn::point_handling::LinearVelocity _velocity;
  double _dof;

 void _init(){
   _axis = _model.mJoints[_frame.getBodyId()].mJointAxes->tail<3>();

   if(_axis.norm() < 0.999)
     _axis = _model.mJoints[_frame.getBodyId()].mJointAxes->head<3>();
   if(_axis.norm() < 0.999)
     throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Unknown joint type for body ") + std::string(_model.GetBodyName(_frame.getBodyId())));
   _dof = _model.mJoints[_frame.getBodyId()].q_index; // this should have chanck on multiple dofs
   std::cout << _model.GetBodyName(_frame.getBodyId()) << " idx " << _dof << std::endl;
 }

};

} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_H
