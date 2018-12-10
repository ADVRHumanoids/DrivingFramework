#ifndef __MWOIBN__HIERARCHICAL_CONTROL__TASKS__CONTACT_POINT_ZMP_H
#define __MWOIBN__HIERARCHICAL_CONTROL__TASKS__CONTACT_POINT_ZMP_H

#include "mwoibn/hierarchical_control/tasks/contact_point_tracking_task.h"
#include "mwoibn/robot_points/torus_model.h"
#include "mwoibn/robot_points/handler.h"
#include "mwoibn/robot_points/center_of_mass.h"
#include "mwoibn/robot_points/rotation.h"
#include "mwoibn/dynamic_points/force.h"
#include "mwoibn/robot_points/ground_wheel.h"
#include <mwoibn/dynamic_models/basic_model.h>


namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{

class ContactPointZMP : public ContactPointTracking
{

public:
/**
 * @param[in] ik the point handler mamber that defines which point is
 **controlled by this task instance it makes a local copy of a point handler to
 **prevent outside user from modifying a controlled point
 *
 * bi_map - bi_map[wrench] = contact
 */
ContactPointZMP(mwoibn::robot_points::Handler<mwoibn::robot_points::TorusModel> contact_points,
                mwoibn::robot_class::Robot& robot, std::string base_link, double position_gain)
        : ContactPointTracking(robot, robot.centerOfPressure(), base_link), _gravity(robot),
          _current_force(_robot.getDofs()), _position_gain(position_gain)
{
  for(auto& contact_: contact_points){
    _wheel_transforms.push_back(std::unique_ptr<mwoibn::robot_points::Rotation>(
        new mwoibn::robot_points::GroundWheel(contact_->axis(), contact_->groundNormal())));
    _contacts.add(std::move(contact_));
  }

  _allocate();

  _gravity.subscribe(mwoibn::robot_class::DYNAMIC_MODEL::NON_LINEAR);
  _robot.state.add("OVERALL_FORCE");

  for(auto& contact: _contacts)
    _current_force.add(mwoibn::dynamic_points::Force(_robot, _gravity, *_minus.end()[-1])); // this should not be state? -- make my own state here?


}

virtual ~ContactPointZMP() { }


//! the force error desried - measure
// need contacts
virtual void updateError(){

    for(int i = 0; i < _contacts.size(); i++)
      _position_error.segment<3>(3*i) =  _q_twist.rotate(_reference.segment<3>(i*3)) - _minus[i].get(); // reference is in the wrong frame

    _full_error = _position_error * _position_gain - _current_force.getState(); // _current_force.getState();

    for(int i = 0; i < _contacts.size(); i++){
      _error.segment<3>(3*i) = _wheel_transforms[i]->rotation.transpose()*(_full_error.segment<3>(3*i));
      _position_error.segment<3>(3*i) = _wheel_transforms[i]->rotation.transpose()*(_position_error.segment<3>(3*i));
      _force.segment<3>(3*i) =  _wheel_transforms[i]->rotation.transpose()*_current_force[i].get();


      if(_selector[i])
        _error.segment<2>(3*i+1).setZero();
    }

}

virtual void updateJacobian(){

    for(int i = 0; i < _contacts.size(); i++){
      _jacobian.block(3*i, 0, 3, _jacobian.cols()) =
          -_wheel_transforms[i]->rotation.transpose()*(_minus[i].getJacobian()); // Definitly a memory allocatino

      if(_selector[i])
          _jacobian.block(3*i+1, 0, 2, _jacobian.cols()).setZero();
      }

}

virtual void updateState(){

  _robot.centerOfMass().update(true);
  _base_point.update(true);

  ContactPointTracking::updateState();

  _gravity.update();

  _robot.state["OVERALL_FORCE"].set(-_gravity.getNonlinearEffects() + _robot.state.torque.get() + _robot.contacts().getWorldJacobian().transpose()*_robot.contacts().getReactionForce());
  _current_force.update(true);

}

  const mwoibn::VectorN& getPositionError(){return _position_error;}

  virtual const mwoibn::Vector3& getPointWorld(int i)
  {
    _point.noalias() = _contacts[i].get();
    return _point;
  }

protected:
  mwoibn::dynamic_models::BasicModel _gravity;

  mwoibn::robot_points::Handler<mwoibn::dynamic_points::Force>  _current_force;

  mwoibn::VectorN _position_error;

  double _position_gain;

  virtual void _allocate(){
    ContactPointTracking::_allocate();
    _position_error.setZero(_contacts.rows());
  }

};
}
} // namespace package
} // namespace library
#endif
