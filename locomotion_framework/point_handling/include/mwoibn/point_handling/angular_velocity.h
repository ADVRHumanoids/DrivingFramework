#ifndef __MWOIBN__POINT_HANDLING__ANGULAR_VELOCITY_H
#define __MWOIBN__POINT_HANDLING__ANGULAR_VELOCITY_H

#include "mwoibn/point_handling/velocity.h"
#include "mwoibn/point_handling/frame_plus.h"

namespace mwoibn
{

namespace point_handling
{

class AngularVelocity: public Velocity
{

public:

  AngularVelocity(point_handling::FramePlus& frame,
       std::string name = "")
      : Velocity(frame, 3, name)
  {
      _J_full.setZero(6, _state.velocity.size()); }

  AngularVelocity(Point::Current current,
        point_handling::FramePlus& frame, std::string name = "")
      : Velocity(current, frame, name)
  {
    _J_full.setZero(6, _state.velocity.size()); }

  AngularVelocity( AngularVelocity&& other)
      : Velocity(other), _J_full(other._J_full)
  {  }

  AngularVelocity(const AngularVelocity& other)
      : Velocity(other), _J_full(other._J_full)
  {  }

  AngularVelocity( AngularVelocity&& other, point_handling::FramePlus& frame)
      : Velocity(other, frame), _J_full(other._J_full)
  {  }

  AngularVelocity(const AngularVelocity& other, point_handling::FramePlus& frame)
      : Velocity(other, frame), _J_full(other._J_full)
  {  }

  virtual ~AngularVelocity() {}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false){
      _temp_world = CalcPointVelocity6D(_model, _state.position.get(), _state.velocity.get(), _body_id, frame.position.getFixed(), false).head<3>();
      return _temp_world;
  }

  virtual Point::Current
  getWorld(bool update = false) const {
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }
  /** @brief set new tracked point giving data in a world frame*/
  virtual void getWorld(Point::Current& current, bool update = false) const {
    current = CalcPointVelocity6D(_model, _state.position.get(), _state.velocity.get(), _body_id, frame.position.getFixed(), false).head<3>();
  }
  /** @brief set new tracked point giving data in a world frame*/

  virtual void setWorld(const Point::Current& current,
                        bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  /** @brief get Position in a user-defined reference frame */
  virtual Point::Current
  getReference(unsigned int refernce_id, bool update = false) const {
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  virtual void getReference(Point::Current& current, unsigned int refernce_id, bool update = false) const {
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  virtual const mwoibn::Matrix& getJacobian(bool update = false);
  virtual mwoibn::Matrix getJacobian(bool update = false) const;
  virtual void getJacobian(mwoibn::Matrix& current, bool update = false) const;

protected:
  mwoibn::Matrix _J_full;

};

} // namespace package
} // namespace library

#endif
