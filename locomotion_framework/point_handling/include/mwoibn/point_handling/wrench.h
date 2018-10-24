#ifndef __MWOIBN__POINT_HANDLING__WRENCH_H
#define __MWOIBN__POINT_HANDLING__WRENCH_H

#include "mwoibn/point_handling/force.h"
#include "mwoibn/point_handling/torque.h"
#include "mwoibn/point_handling/point.h"

namespace mwoibn
{

namespace point_handling
{

class Wrench: public State
{

public:

  template<typename Type>
  Wrench(Type body_id, RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        point_handling::Position& position, point_handling::Rotation& rotation, std::string name = "")
      : State(body_id, model, state, position, 6, name), _rotation(rotation),
       _force(body_id, model, state, position, rotation, name), _torque(body_id, model, state, _force, name)
  {
  }

  template<typename Type>
  Wrench(Point::Current current, Type body_id,
         RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
         point_handling::Position& position, point_handling::Rotation& rotation, std::string name = "")
      : State(current, body_id, model, state, position, 6, name), _rotation(rotation),
       _force(body_id, model, state, position, rotation, name), _torque(body_id, model, state, _force, name)
  {
      synch();
  }

  Wrench(const Wrench&& other) : State(other),
        _rotation(other._rotation),
        _force(other._force, _position, _rotation), _torque(other._torque, _force)
  {  }

  Wrench(const Wrench& other) : State(other),
        _rotation(other._rotation), _force(other._force, _position, _rotation), _torque(other._torque, _force)
  {  }

  Wrench(const Wrench&& other, point_handling::Position& position, point_handling::Rotation& rotation) : State(other, position),
        _rotation(rotation), _force(other._force, _position, _rotation), _torque(other._torque, _force)
  {  }

  Wrench(const Wrench& other, point_handling::Position& position, point_handling::Rotation& rotation) : State(other, position),
        _rotation(rotation), _force(other._force, _position, _rotation), _torque(other._torque, _force)
  {  }


  virtual ~Wrench() {}

  virtual const Point::Current&
  getWorld(bool update = false);


  virtual Point::Current
  getWorld(bool update = false) const;

  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& linear,
                        bool update = false);

  virtual void setFixed(const Point::Current& current);


  /** @brief get Position in a user-defined reference frame */
  virtual const Point::Current&
  getReference(unsigned int refernce_id, bool update = false);

  virtual void setReference(const Point::Current& position,
                            unsigned int reference_id,
                            bool update = false);

  Force& force(){return _force;}
  Torque& torque(){return _torque;}
  Rotation& rotation(){return _rotation;}

  const Force& force() const {return _force;}
  const Torque& torque() const {return _torque;}
  const Rotation& rotation() const {return _rotation;}

  void synch();

  protected:
    Rotation& _rotation;
    Force _force;
    Torque _torque;

    void _get(Point::Current& current, const Point::Current& force, const Point::Current& torque) const;

};

} // namespace package
} // namespace library

#endif
