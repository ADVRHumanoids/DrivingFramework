#ifndef __MWOIBN__POINT_HANDLING__SPATIAL_VELOCITY_H
#define __MWOIBN__POINT_HANDLING__SPATIAL_VELOCITY_H

#include "mwoibn/point_handling/point.h"
#include "mwoibn/point_handling/velocity.h"

#include "mwoibn/point_handling/angular_velocity.h"
#include "mwoibn/point_handling/linear_velocity.h"

namespace mwoibn
{

namespace point_handling
{

class SpatialVelocity: public Velocity
{

public:


  template<typename Body>
  SpatialVelocity(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, point_handling::Position& position, std::string name = "")
      : Velocity(body_id, model, state, position, 6, name),
       _linear(body_id, model, state, position, name), _angular(body_id, model, state, position, name)
  {
  }

  template<typename Body>
  SpatialVelocity(Point::Current current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        point_handling::Position& position, std::string name = "")
      : Velocity(current, body_id, model, state, position, 6, name),
       _linear(body_id, model, state, position, name), _angular(body_id, model, state, position, name)
  {
    _current.noalias() = current;
    _linear.setFixed(current.tail<3>());
    _angular.setFixed(current.head<3>());
   }

  SpatialVelocity(const SpatialVelocity&& other)
      : Velocity(other), _linear(other._linear, _position), _angular(other._angular, _position)
  {  }

  SpatialVelocity(const SpatialVelocity& other)
      : Velocity(other), _linear(other._linear, _position), _angular(other._angular, _position)
  {  }

  SpatialVelocity(const SpatialVelocity&& other, point_handling::Position& position)
      : Velocity(other, position), _linear(other._linear, _position), _angular(other._angular, _position)
  {  }

  SpatialVelocity(const SpatialVelocity& other, point_handling::Position& position)
      : Velocity(other, position), _linear(other._linear, _position), _angular(other._angular, _position)
  {  }

  virtual ~SpatialVelocity() {}

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

  LinearVelocity& linear(){return _linear;} // add synchronization mechanism
  AngularVelocity& angular(){return _angular;} // add synchronization mechanism

  const LinearVelocity& linear() const {return _linear;} // add synchronization mechanism
  const AngularVelocity& angular() const {return _angular;} // add synchronization mechanism

protected:
  LinearVelocity _linear;
  AngularVelocity _angular;


};

} // namespace package
} // namespace library

#endif
