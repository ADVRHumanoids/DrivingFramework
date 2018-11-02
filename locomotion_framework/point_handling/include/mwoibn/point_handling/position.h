#ifndef __MWOIBN__POINT_HANDLING__POSITION_H
#define __MWOIBN__POINT_HANDLING__POSITION_H

#include "mwoibn/point_handling/point.h"

namespace mwoibn
{

namespace point_handling
{

class Position: public Point
{

public:

  template<typename Body>
  Position(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, std::string name = "")
      : Point(body_id, model, state, 3, name)
  {  }

  // Point(std::string body_name, RigidBodyDynamics::Model& model,
  //      const mwoibn::robot_class::State& state, std::string name = "")
  //     : Base(body_name, model, state, name)
  // {
  //   _current.setZero(size);
  // }
  template<typename Body>
  Position(Point::Current current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        std::string name = "")
      : Point(current, body_id, model, state, 3, name)
  {  }

  // Base(Type current, std::string body_name,
  //       RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
  //       std::string name = "")
  //     : _name(name), _model(model), _body_id(_checkBody(body_name, model)),
  //        _current(current), _state(state)
  // {  }

  Position(const Position&& other)
      : Point(other)
  {  }

  Position(const Position& other)
      : Point(other)
  {  }

  virtual ~Position() {}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false);

  virtual Point::Current
  getWorld(bool update = false) const;

  virtual void
  getWorld(Point::Current& current, bool update = false) const;

  virtual void
  getWorld(mwoibn::Vector3& current, bool update = false) const;

  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false);

  /** @brief get Position in a user-defined reference frame */
  virtual const Point::Current&
  getReference(unsigned int refernce_id, bool update = false);

  virtual void
  getReference(Point::Current& current, unsigned int refernce_id, bool update = false) const;

  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false);


};

} // namespace package
} // namespace library

#endif
