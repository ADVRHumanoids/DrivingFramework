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


  template<typename Body>
  Torque(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, point_handling::Force& force, std::string name = "")
      : State(body_id, model, state, force.position(), 3, name), _rotation(force.rotation()), _force(force)
  {  }

  template<typename Body>
  Torque(Point::Current current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        point_handling::Force& force, std::string name = "")
      : State(current, body_id, model, state, force.position(), 3, name), _rotation(force.rotation()), _force(force)
  { }

  Torque(const Torque&& other)
      : State(other), _rotation(other._rotation), _force(other._force)
  {  }

  Torque(const Torque& other)
      : State(other), _rotation(other._rotation), _force(other._force)
  {  }

  Torque(const Torque&& other, point_handling::Force& force)
      : State(other, force.position()), _rotation(force.rotation()), _force(force)
  {  }

  Torque(const Torque& other, point_handling::Force& force)
      : State(other, force.position()), _rotation(force.rotation()), _force(force)
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

  Force& force() {return _force;}

  protected:
    Rotation& _rotation;
    Force& _force;

};

} // namespace package
} // namespace library

#endif
