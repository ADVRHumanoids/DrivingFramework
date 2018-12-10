#ifndef __MWOIBN__POINT_HANDLING__WRENCH_H
#define __MWOIBN__POINT_HANDLING__WRENCH_H

#include "mwoibn/point_handling/force.h"
#include "mwoibn/point_handling/torque.h"
#include "mwoibn/point_handling/frame_plus.h"

namespace mwoibn
{

namespace point_handling
{

class Wrench: public State
{

public:

  Wrench(point_handling::FramePlus& frame, std::string name = "")
      : State(frame, 6, name), force(frame, name), torque(force, name)
  {
    _temp.setZero(3);
  }

  Wrench(Point::Current current, point_handling::FramePlus& frame, std::string name = "")
      : State(current, frame, name), force(frame, name), torque(force, name)
  {
    _temp.setZero(3);
      synch();
      _size = 6;
  }

  Wrench( Wrench&& other) : State(other),
        force(other.force, frame), torque(other.torque, force), _temp(other._temp)
  {  }

  Wrench(const Wrench& other) : State(other), force(other.force, frame), torque(other.torque, force), _temp(other._temp)
  {  }

  Wrench( Wrench&& other, point_handling::FramePlus& frame) : State(other, frame), force(other.force, frame), torque(other.torque, force), _temp(other._temp)
  {  }

  Wrench(const Wrench& other, point_handling::FramePlus& frame) : State(other, frame), force(other.force, frame), torque(other.torque, force), _temp(other._temp)
  {  }



  virtual ~Wrench() {}

  virtual const Point::Current&
  getWorld(bool update = false);


  virtual Point::Current
  getWorld(bool update = false) const;

  virtual void
  getWorld(Point::Current& current, bool update = false) const;

  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& linear,
                        bool update = false);

  virtual void setFixed(const Point::Current& current);
  virtual void setFixed(const mwoibn::Vector6& current);


  /** @brief get Position in a user-defined reference frame */
  virtual Point::Current
  getReference(unsigned int refernce_id, bool update = false) const;

  virtual void
  getReference(Point::Current& current, unsigned int refernce_id, bool update = false) const;

  virtual void setReference(const Point::Current& frame,
                            unsigned int reference_id,
                            bool update = false);

  Force force;
  Torque torque;
  Point::Current _temp;
  void synch();

  protected:

    void _get(Point::Current& current, const Point::Current& force, const Point::Current& torque) const;

};

} // namespace package
} // namespace library

#endif
