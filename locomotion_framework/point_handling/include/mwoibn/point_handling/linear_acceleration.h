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

  LinearAcceleration(point_handling::FramePlus& frame, std::string name = "")
      : State(frame, 3, name)
  {  }

  LinearAcceleration(Point::Current current, point_handling::FramePlus& frame, std::string name = "")
      : State(current, frame, name)
  { _size = 3; }

  LinearAcceleration(const LinearAcceleration&& other)
      : State(other)
  {  }

  LinearAcceleration(const LinearAcceleration& other)
      : State(other)
  {  }

  LinearAcceleration(const LinearAcceleration&& other, point_handling::FramePlus& frame)
      : State(other, frame)
  {  }

  LinearAcceleration(const LinearAcceleration& other, point_handling::FramePlus& frame)
      : State(other, frame)
  {  }

  virtual ~LinearAcceleration() {}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false){
      _temp_current = CalcPointAcceleration(_model, _state.get(INTERFACE::POSITION), _state.get(INTERFACE::VELOCITY),
                                                    _state.get(INTERFACE::ACCELERATION), _body_id, frame.position.getFixed());

      _temp_current += frame.rotation().getWorld()*_current;
      return _temp_current;

  } // NOT IMPLEMENTED

  virtual Point::Current
  getWorld(bool update = false) const {
    return CalcPointAcceleration(_model, _state.get(INTERFACE::POSITION), _state.get(INTERFACE::VELOCITY),
                                         _state.get(INTERFACE::ACCELERATION), _body_id, frame.position.getFixed()) + frame.rotation().getWorld()*_current;
  } // NOT IMPLEMENTED

  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false){}// NOT IMPLEMENTED

  /** @brief get Position in a user-defined reference frame */
  virtual const Point::Current&
  getReference(unsigned int refernce_id, bool update = false){}// NOT IMPLEMENTED

  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false){}// NOT IMPLEMENTED


  protected:
    using INTERFACE = mwoibn::robot_class::INTERFACE;

};

} // namespace package
} // namespace library

#endif
