#ifndef __MWOIBN__POINT_HANDLING__ANGULAR_ACCELRATION_H
#define __MWOIBN__POINT_HANDLING__ANGULAR_ACCELRATION_H

#include "mwoibn/point_handling/frame_plus.h"

namespace mwoibn
{

namespace point_handling
{

class AngularAcceleration: public State
{

public:

  AngularAcceleration(point_handling::FramePlus& frame, mwoibn::Interface acceleration_interface = "ACCELERATION",
       std::string name = "")
      : State(frame, 3, name), _acceleration_interface(acceleration_interface)
  { }

  AngularAcceleration(const Point::Current& current,
        point_handling::FramePlus& frame, mwoibn::Interface acceleration_interface = "ACCELERATION", std::string name = "")
      : State(current, frame, name), _acceleration_interface(acceleration_interface)
  { }

  AngularAcceleration( AngularAcceleration&& other)
      : State(other), _acceleration_interface(other._acceleration_interface)
  {  }

  AngularAcceleration(const AngularAcceleration& other)
      : State(other), _acceleration_interface(other._acceleration_interface)
  {  }

  AngularAcceleration( AngularAcceleration&& other, point_handling::FramePlus& frame)
      : State(other, frame), _acceleration_interface(other._acceleration_interface)
  {  }

  AngularAcceleration(const AngularAcceleration& other, point_handling::FramePlus& frame)
      : State(other, frame), _acceleration_interface(other._acceleration_interface)
  {  }

  virtual ~AngularAcceleration() {}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false){
      _temp_world = CalcPointAcceleration6D(_model, _state.position.get(), _state.velocity.get(), _state[_acceleration_interface].get(), _body_id, frame.position.getFixed(), false).head<3>();
      return _temp_world;
  }

  // virtual Point::Current
  // getWorld(bool update = false) const {
  //   throw mwoibn::std_utils::deprecated(__PRETTY_FUNCTION__);
  // }
  /** @brief set new tracked point giving data in a world frame*/
  virtual void getWorld(Point::Current& current, bool update = false) const {
    current = CalcPointAcceleration6D(_model, _state.position.get(), _state.velocity.get(), _state[_acceleration_interface].get(), _body_id, frame.position.getFixed(), false).head<3>();
  }
  /** @brief set new tracked point giving data in a world frame*/

  virtual void setWorld(const Point::Current& current,
                        bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  /** @brief get Position in a user-defined reference frame */
  // virtual Point::Current
  // getReference(unsigned int refernce_id, bool update = false) const {
  //   throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  // }

  virtual void getReference(Point::Current& current, unsigned int refernce_id, bool update = false) const {
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

protected:
  Interface _acceleration_interface;


};

} // namespace package
} // namespace library

#endif
