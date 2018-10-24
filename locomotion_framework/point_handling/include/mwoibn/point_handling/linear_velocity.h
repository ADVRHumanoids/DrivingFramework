#ifndef __MWOIBN__POINT_HANDLING__LINEAR_VELOCITY_H
#define __MWOIBN__POINT_HANDLING__LINEAR_VELOCITY_H

#include "mwoibn/point_handling/velocity.h"
#include "mwoibn/point_handling/position.h"

namespace mwoibn
{

namespace point_handling
{

class LinearVelocity: public Velocity
{

public:


  template<typename Body>
  LinearVelocity(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, point_handling::Position& position, std::string name = "")
      : Velocity(body_id, model, state, position, 3, name)
  {  }

  template<typename Body>
  LinearVelocity(Point::Current current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        point_handling::Position& position, std::string name = "")
      : Velocity(current, body_id, model, state, position, 3, name)
  { }

  LinearVelocity(const LinearVelocity&& other)
      : Velocity(other)
  {  }

  LinearVelocity(const LinearVelocity& other)
      : Velocity(other)
  {  }

  LinearVelocity(const LinearVelocity&& other, point_handling::Position& position)
      : Velocity(other, position)
  {  }

  LinearVelocity(const LinearVelocity& other, point_handling::Position& position)
      : Velocity(other, position)
  {  }

  virtual ~LinearVelocity() {}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false){} // NOT IMPLEMENTED

  virtual Point::Current
  getWorld(bool update = false) const {} // NOT IMPLEMENTED
  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false){}// NOT IMPLEMENTED

  /** @brief get Position in a user-defined reference frame */
  virtual const Point::Current&
  getReference(unsigned int refernce_id, bool update = false){}// NOT IMPLEMENTED

  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false){}// NOT IMPLEMENTED

  const mwoibn::Matrix& getJacobian(bool update = false);
  mwoibn::Matrix getJacobian(bool update = false) const;

};

} // namespace package
} // namespace library

#endif
