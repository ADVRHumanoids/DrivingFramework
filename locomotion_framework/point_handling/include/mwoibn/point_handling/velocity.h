#ifndef __MWOIBN__POINT_HANDLING__VELOCITY_H
#define __MWOIBN__POINT_HANDLING__VELOCITY_H

#include "mwoibn/point_handling/state.h"
#include "mwoibn/point_handling/frame_plus.h"

namespace mwoibn
{

namespace point_handling
{

class Velocity: public State
{

public:


  Velocity(point_handling::FramePlus& frame, int size, std::string name = "")
      : State(frame, size, name)
  {
    _J.setZero(size, _state.velocity.size());
  }

  Velocity(Point::Current& current,
        point_handling::FramePlus& frame, std::string name = "")
      : State(current, frame, name)

  { _J.setZero(3, _state.velocity.size());
    _size = 3;
  }

  Velocity(const Velocity&& other)
      : State(other), _J(other._J)
  {  }

  Velocity(const Velocity& other)
      : State(other), _J(other._J)
  {  }


  Velocity(const Velocity&& other, point_handling::FramePlus& frame)
        : State(other, frame), _J(other._J)
    {  }

  Velocity(const Velocity& other, point_handling::FramePlus& frame)
        : State(other, frame), _J(other._J)
    {  }

  virtual ~Velocity() {}


  virtual const mwoibn::Matrix& getJacobian(bool update = false) = 0;
  virtual mwoibn::Matrix getJacobian(bool update = false) const = 0;
  virtual void getJacobian(mwoibn::Matrix& current, bool update = false) const = 0;



protected:
  mwoibn::Matrix _J;

};

} // namespace package
} // namespace library

#endif
