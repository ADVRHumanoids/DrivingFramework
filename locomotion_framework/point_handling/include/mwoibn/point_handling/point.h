#ifndef __MWOIBN__POINT_HANDLING__POINT_H
#define __MWOIBN__POINT_HANDLING__POINT_H

#include "mwoibn/point_handling/base.h"

namespace mwoibn
{

namespace point_handling
{


class Point: public Base<mwoibn::VectorN>
{

public:
  typedef mwoibn::VectorN Current;

  template<typename Body>
  Point(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, int size, std::string name = "")
      : Base(body_id, model, state, size, name)
  {
    _current.setZero(size);
    _temp_current.setZero(size);
  }

  // Point(std::string body_name, RigidBodyDynamics::Model& model,
  //      const mwoibn::robot_class::State& state, std::string name = "")
  //     : Base(body_name, model, state, name)
  // {
  //   _current.setZero(size);
  // }
  template<typename Body>
  Point(Point::Current current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        std::string name = "")
      : Base(current, body_id, model, state, current.size(), name)
  {}

  // Base(Type current, std::string body_name,
  //       RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
  //       std::string name = "")
  //     : _name(name), _model(model), _body_id(_checkBody(body_name, model)),
  //        _current(current), _state(state)
  // {  }

  Point(const Point&& other)
      : Base(other)
  {  }

  Point(const Point& other)
      : Base(other)
  {  }


    template<typename Source>
    Point(const Source&& other, int size, std::string name = "")
        : Base(other, size, name)
    {     _current.setZero(size);
        _temp_current.setZero(size); }

    template<typename Source>
    Point(const Source& other, int size, std::string name = "")
        : Base(other, size, name)
    {     _current.setZero(size);
        _temp_current.setZero(size); }

    template<typename Source>
    Point(Point::Current current, const Source&& other, std::string name = "")
        : Base(other, current.size(), name)
    {  }

    template<typename Source>
    Point(Point::Current current, const Source& other, std::string name = "")
        : Base(other, current.size(), name)
    {  }


  using Base::getReference;
  using Base::setReference;

  ~Point() {}

};

} // namespace package
} // namespace library

#endif
