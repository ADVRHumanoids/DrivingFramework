#ifndef __MWOIBN__POINT_HANDLING__LINEAR_ACCELERATION_H
#define __MWOIBN__POINT_HANDLING__LINEAR_ACCELERATION_H

#include "mwoibn/point_handling/state.h"
#include "mwoibn/point_handling/frame_plus.h"

namespace mwoibn
{

namespace point_handling
{

class LinearAcceleration: public State
{

public:

  LinearAcceleration(point_handling::FramePlus& frame, mwoibn::Interface acceleration_interface = "ACCELERATION", std::string name = "")
      : State(frame, 3, name), _acceleration_interface(acceleration_interface)
  {  }

  LinearAcceleration(const Point::Current& current, point_handling::FramePlus& frame, mwoibn::Interface acceleration_interface = "ACCELERATION", std::string name = "")
      : State(current, frame, name), _acceleration_interface(acceleration_interface)
  { _size = 3; }

  LinearAcceleration( LinearAcceleration&& other)
      : State(other), _acceleration_interface(other._acceleration_interface)
  {  }

  LinearAcceleration(const LinearAcceleration& other)
      : State(other), _acceleration_interface(other._acceleration_interface)
  {  }

  LinearAcceleration( LinearAcceleration&& other, point_handling::FramePlus& frame)
      : State(other, frame), _acceleration_interface(other._acceleration_interface)
  {  }

  LinearAcceleration(const LinearAcceleration& other, point_handling::FramePlus& frame)
      : State(other, frame), _acceleration_interface(other._acceleration_interface)
  {  }

  virtual ~LinearAcceleration() {}

  virtual LinearAcceleration* clone_impl() const {return new LinearAcceleration(*this);}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false){
      _temp_world = CalcPointAcceleration(_model, _state.position.get(), _state.velocity.get(),
                                  _state[_acceleration_interface].get(), _body_id, frame.position.getFixed());

      //_temp_current += frame.rotation().getWorld()*_current_fixed;
      return _temp_world;

  }
  //
  // virtual Point::Current
  // getWorld(bool update = false) const {
  //   return CalcPointAcceleration(_model, _state.position.get(), _state.velocity.get(),
  //                                        _state[_acceleration_interface].get(), _body_id, frame.position.getFixed()) + frame.rotation().getWorld()*_current_fixed;
  // }

  virtual void
  getWorld(Point::Current& current, bool update = false) const {
    current.head<3>() =  CalcPointAcceleration(_model, _state.position.get(), _state.velocity.get(),
                                         _state[_acceleration_interface].get(), _body_id, frame.position.getFixed()) + frame.rotation().getWorld()*_current_fixed;
  }

  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }
  //
  // /** @brief get Position in a user-defined reference frame */
  // virtual Point::Current
  // getReference(unsigned int refernce_id, bool update = false) const {
  //   throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  // }

  virtual void
  getReference(Point::Current& current, unsigned int refernce_id, bool update = false) const {
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
