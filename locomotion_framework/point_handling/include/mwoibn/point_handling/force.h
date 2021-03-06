#ifndef __MWOIBN__POINT_HANDLING__FORCE_H
#define __MWOIBN__POINT_HANDLING__FORCE_H

#include "mwoibn/point_handling/state.h"
#include "mwoibn/point_handling/frame_plus.h"

namespace mwoibn
{

namespace point_handling
{

// Computes the actuation torques component to excert given force at the point
class Force: public State
{

public:


  Force(point_handling::FramePlus& frame, std::string name = "")
      : State(frame, 3, name)
  {  }

  Force(const Point::Current& current,
       point_handling::FramePlus& frame, std::string name = "")
      : State(current, frame, name)
  {
      _size = 3;
  }

  Force( Force&& other,point_handling::FramePlus& frame)
      : State(other, frame)
  {  }

  Force(const Force& other,point_handling::FramePlus& frame)
      : State(other, frame)
  {  }

  Force( Force&& other)
      : State(other)
  {  }

  Force(const Force& other)
      : State(other)
  {  }

  virtual ~Force() {}

  virtual Force* clone_impl() const {return new Force(*this);}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false);

  // virtual Point::Current
  // getWorld(bool update = false) const;

  virtual void
  getWorld(Point::Current& current, bool update = false) const;

  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false);

  /** @brief get Position in a user-defined reference frame */
  // virtual Point::Current
  // getReference(unsigned int refernce_id, bool update = false) const;

  virtual void
  getReference(Point::Current& current, unsigned int refernce_id, bool update = false) const;

  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false);

  using State::getReference;
  using State::setReference;


};

} // namespace package
} // namespace library

#endif
