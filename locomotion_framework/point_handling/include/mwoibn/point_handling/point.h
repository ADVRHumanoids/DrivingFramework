
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
    _current_fixed.setZero(size);
    _temp_world.setZero(size);
  }

  template<typename Body>
  Point(Point::Current current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        std::string name = "")
      : Base(current, body_id, model, state, current.size(), name)
  {
  }

  Point( Point&& other)
      : Base(other)
  {  }

  Point(const Point& other)
      : Base(other)
  {  }

    template<typename Source>
    Point( Source&& other, int size, std::string name = "")
        : Base(other, size, name)
    {     _current_fixed.setZero(size);
        _temp_world.setZero(size); }

    template<typename Source>
    Point(const Source& other, int size, std::string name = "")
        : Base(other, size, name)
    {     _current_fixed.setZero(size);
        _temp_world.setZero(size); }

    template<typename Source>
    Point(Point::Current current,  Source&& other, std::string name = "")
        : Base(other, current.size(), name)
    {  }

    template<typename Source>
    Point(Point::Current current, const Source& other, std::string name = "")
        : Base(other, current.size(), name)
    {  }

    virtual void getWorld(Point::Current& current, bool update = false) const = 0;

    virtual void getReference(Point::Current& current, unsigned int refernce_id, bool update = false) const = 0;
    virtual void setFixed(const Point::Current& current){ _current_fixed.noalias() = current; }


  using Base::getReference;
  using Base::setReference;



  ~Point() {}

};

} // namespace package
} // namespace library

#endif
