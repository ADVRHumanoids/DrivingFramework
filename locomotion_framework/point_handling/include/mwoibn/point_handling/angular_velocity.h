#ifndef __MWOIBN__POINT_HANDLING__ANGULAR_VELOCITY_H
#define __MWOIBN__POINT_HANDLING__ANGULAR_VELOCITY_H

#include "mwoibn/point_handling/velocity.h"
#include "mwoibn/point_handling/position.h"

namespace mwoibn
{

namespace point_handling
{

class AngularVelocity: public Velocity
{

public:


  template<typename Body>
  AngularVelocity(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, point_handling::Position& position,
       std::string name = "")
      : Velocity(body_id, model, state, position, 3, name)
  {
      _J_full.setZero(6, state.size()); }

  template<typename Body>
  AngularVelocity(Point::Current current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        point_handling::Position& position, std::string name = "")
      : Velocity(current, body_id, model, state, position, 3, name)
  {
    _J_full.setZero(6, state.size()); }

  AngularVelocity(const AngularVelocity&& other)
      : Velocity(other), _J_full(other._J_full)
  {  }

  AngularVelocity(const AngularVelocity& other)
      : Velocity(other), _J_full(other._J_full)
  {  }

  AngularVelocity(const AngularVelocity&& other, point_handling::Position& position)
      : Velocity(other, position), _J_full(other._J_full)
  {  }

  AngularVelocity(const AngularVelocity& other, point_handling::Position& position)
      : Velocity(other, position), _J_full(other._J_full)
  {  }

  virtual ~AngularVelocity() {}

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

protected:
  mwoibn::Matrix _J_full;

};

} // namespace package
} // namespace library

#endif
