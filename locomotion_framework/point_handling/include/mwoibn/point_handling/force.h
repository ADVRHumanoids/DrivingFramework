#ifndef __MWOIBN__POINT_HANDLING__FORCE_H
#define __MWOIBN__POINT_HANDLING__FORCE_H

#include "mwoibn/point_handling/state.h"
#include "mwoibn/point_handling/position.h"
#include "mwoibn/point_handling/rotation.h"

namespace mwoibn
{

namespace point_handling
{

class Force: public State
{

public:


  template<typename Body>
  Force(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, point_handling::Position& position, point_handling::Rotation& rotation, std::string name = "")
      : State(body_id, model, state, position, 3, name), _rotation(rotation)
  {  }

  template<typename Body>
  Force(Point::Current current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        point_handling::Position& position, point_handling::Rotation& rotation, std::string name = "")
      : State(current, body_id, model, state, position, 3, name), _rotation(rotation)
  { }

  Force(const Force&& other, point_handling::Position& position, point_handling::Rotation& rotation)
      : State(other, position), _rotation(rotation)
  {  }

  Force(const Force& other, point_handling::Position& position, point_handling::Rotation& rotation)
      : State(other, position), _rotation(rotation)
  {  }

  Force(const Force&& other)
      : State(other), _rotation(other._rotation)
  {  }

  Force(const Force& other)
      : State(other), _rotation(other._rotation)
  {  }

  virtual ~Force() {}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false);

  virtual Point::Current
  getWorld(bool update = false) const;
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

  Rotation& rotation(){return _rotation;}

  protected:
    Rotation& _rotation;

};

} // namespace package
} // namespace library

#endif
