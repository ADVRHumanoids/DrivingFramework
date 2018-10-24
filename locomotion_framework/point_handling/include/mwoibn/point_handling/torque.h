#ifndef __MWOIBN__POINT_HANDLING__TORQUE_H
#define __MWOIBN__POINT_HANDLING__TORQUE_H

#include "mwoibn/point_handling/state.h"
#include "mwoibn/point_handling/force.h"

namespace mwoibn
{

namespace point_handling
{

class Torque: public State
{

public:


  Torque(point_handling::Force& force, std::string name = "")
      : State(force.frame, 3, name), force(force)
  {  }

  Torque(Point::Current current,
        point_handling::Force& force, std::string name = "")
      : State(current, force.frame, name), force(force)
  {
    _size = 3;
  }

  Torque(const Torque&& other)
      : State(other), force(other.force)
  {  }

  Torque(const Torque& other)
      : State(other), force(other.force)
  {  }

  Torque(const Torque&& other, point_handling::Force& force)
      : State(other, force.frame), force(force)
  {  }

  Torque(const Torque& other, point_handling::Force& force)
      : State(other, force.frame), force(force)
  {  }

  virtual ~Torque() {}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false);

  virtual Point::Current
  getWorld(bool update = false) const;

  void getWorld(Point::Current& angular, bool update) const;


  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false);

  /** @brief get Position in a user-defined reference frame */
  virtual const Point::Current&
  getReference(unsigned int refernce_id, bool update = false);

  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false);

  using State::getReference;
  using State::setReference;

  Force& force;

};

} // namespace package
} // namespace library

#endif
