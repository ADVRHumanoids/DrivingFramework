#ifndef __MWOIBN__POINT_HANDLING__STATE_H
#define __MWOIBN__POINT_HANDLING__STATE_H

#include "mwoibn/point_handling/point.h"
#include "mwoibn/point_handling/position.h"

namespace mwoibn
{

namespace point_handling
{

class State: public Point
{

public:


  template<typename Body>
  State(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, point_handling::Position& position, int size, std::string name = "")
      : Point(body_id, model, state, size, name), _position(position)
  {
  }

  template<typename Body>
  State(Point::Current current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        point_handling::Position& position, int size, std::string name = "")
      : Point(current, body_id, model, state, name), _position(position)
  { }

  State(const State&& other)
      : Point(other), _position(other._position)
  {  }

  State(const State& other)
      : Point(other), _position(other._position)
  {  }

  State(const State&& other, point_handling::Position& position)
      : Point(other), _position(position)
  {  }

  State(const State& other, point_handling::Position& position)
      : Point(other), _position(position)
  {  }

  virtual ~State() {}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false) = 0; // NOT IMPLEMENTED

  virtual Point::Current
  getWorld(bool update = false) const = 0; // NOT IMPLEMENTED
  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false) = 0;// NOT IMPLEMENTED

  /** @brief get Position in a user-defined reference frame */
  virtual const Point::Current&
  getReference(unsigned int refernce_id, bool update = false) = 0;// NOT IMPLEMENTED

  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false) = 0;// NOT IMPLEMENTED

  using Point::getReference;
  using Point::setReference;

  Position& position(){return _position;}

protected:
  Position& _position;

};

} // namespace package
} // namespace library

#endif
