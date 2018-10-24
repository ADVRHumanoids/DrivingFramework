#ifndef __MWOIBN__POINT_HANDLING__VELOCITY_H
#define __MWOIBN__POINT_HANDLING__VELOCITY_H

#include "mwoibn/point_handling/state.h"
#include "mwoibn/point_handling/position.h"

namespace mwoibn
{

namespace point_handling
{

class Velocity: public State
{

public:


  template<typename Body>
  Velocity(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, point_handling::Position& position, int size, std::string name = "")
      : State(body_id, model, state, position, size, name)
  {
    _J.setZero(size, state.size());
  }

  template<typename Body>
  Velocity(Point::Current current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        point_handling::Position& position, int size, std::string name = "")
      : State(current, body_id, model, state, position, size, name)
  { _J.setZero(size, state.size()); }

  Velocity(const Velocity&& other)
      : State(other), _J(other._J)
  {  }

  Velocity(const Velocity& other)
      : State(other), _J(other._J)
  {  }


  Velocity(const Velocity&& other, point_handling::Position& position)
        : State(other, position), _J(other._J)
    {  }

  Velocity(const Velocity& other, point_handling::Position& position)
        : State(other, position), _J(other._J)
    {  }

  virtual ~Velocity() {}

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

  virtual const mwoibn::Matrix& getJacobian(bool update = false) = 0;
  virtual mwoibn::Matrix getJacobian(bool update = false) const = 0;



protected:
  mwoibn::Matrix _J;

};

} // namespace package
} // namespace library

#endif
