#ifndef __MWOIBN__POINT_HANDLING__STATE_H
#define __MWOIBN__POINT_HANDLING__STATE_H

#include "mwoibn/point_handling/point.h"
#include "mwoibn/point_handling/frame_plus.h"

namespace mwoibn
{

namespace point_handling
{

class State: public Point
{

public:

  State(point_handling::FramePlus& frame, int size, std::string name = "")
      : Point(frame, size, name), frame(frame)
  {
  }

  State(const Point::Current& current, point_handling::FramePlus& frame, std::string name = "")
      : Point(current, frame, name), frame(frame)
  { }

  State( State&& other)
      : Point(other), frame(other.frame)
  {  }

  State(const State& other)
      : Point(other), frame(other.frame)
  {  }

  State( State&& other, point_handling::FramePlus& frame)
      : Point(other), frame(frame)
  {  }

  State(const State& other, point_handling::FramePlus& frame)
      : Point(other), frame(frame)
  {  }

  virtual ~State() {}

  // virtual State* clone_impl() const {return new State(*this);}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false) = 0;

  // virtual Point::Current
  // getWorld(bool update = false) const = 0;
  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false) = 0;

  // /** @brief get Position in a user-defined reference frame */
  // virtual Point::Current
  // getReference(unsigned int refernce_id, bool update = false) const = 0;

  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false) = 0;

  using Point::getReference;
  using Point::setReference;

  Rotation& rotation(){return frame.rotation();}
  Position& position(){return frame.position;}
  FramePlus& frame;


};

} // namespace package
} // namespace library

#endif
