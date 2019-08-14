#ifndef __MWOIBN__POINT_HANDLING__SPATIAL_ACCELERATION_H
#define __MWOIBN__POINT_HANDLING__SPATIAL_ACCELERATION_H

#include "mwoibn/point_handling/state.h"
#include "mwoibn/point_handling/frame_plus.h"

#include "mwoibn/point_handling/angular_acceleration.h"
#include "mwoibn/point_handling/linear_acceleration.h"

namespace mwoibn
{

namespace point_handling
{

class SpatialAcceleration: public State
{

public:

  SpatialAcceleration(point_handling::FramePlus& frame, mwoibn::Interface acceleration_interface = "ACCELERATION", std::string name = "")
      : State(frame, 6, name), _acceleration_interface(acceleration_interface), _linear(frame, name), _angular(frame, name)
  {  }

  SpatialAcceleration(const Point::Current& current, point_handling::FramePlus& frame, mwoibn::Interface acceleration_interface = "ACCELERATION", std::string name = "")
      : State(current, frame, name), _acceleration_interface(acceleration_interface), _linear(frame, name), _angular(frame, name)
  { _size = 6; }

  SpatialAcceleration( SpatialAcceleration&& other)
      : State(other), _acceleration_interface(other._acceleration_interface), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  SpatialAcceleration(const SpatialAcceleration& other)
      : State(other), _acceleration_interface(other._acceleration_interface), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  SpatialAcceleration( SpatialAcceleration&& other, point_handling::FramePlus& frame)
      : State(other, frame), _acceleration_interface(other._acceleration_interface), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  SpatialAcceleration(const SpatialAcceleration& other, point_handling::FramePlus& frame)
      : State(other, frame), _acceleration_interface(other._acceleration_interface), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  virtual ~SpatialAcceleration() {}

  virtual SpatialAcceleration* clone_impl() const {return new SpatialAcceleration(*this);}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false){
      _temp_world = CalcPointAcceleration6D(_model, _state.position.get(), _state.velocity.get(),
                                   _state[_acceleration_interface].get(), _body_id, frame.position.getFixed());
      //
      // //_temp_current += frame.rotation().getWorld()*_current_fixed;
      return _temp_world;
      //throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }
  //
  // virtual Point::Current
  // getWorld(bool update = false) const {
  //   return CalcPointAcceleration(_model, _state.position.get(), _state.Acceleration.get(),
  //                                        _state[_acceleration_interface].get(), _body_id, frame.position.getFixed()) + frame.rotation().getWorld()*_current_fixed;
  // }

  virtual void
  getWorld(Point::Current& current, bool update = false) const {
    current = CalcPointAcceleration6D(_model, _state.position.get(), _state.velocity.get(),
                                 _state[_acceleration_interface].get(), _body_id, frame.position.getFixed());
  }

  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }
  //
  // /** @brief get Position in a user-defined reference frame */
  // virtual Point::Current
  // getReference(unsigned int refernce_id, bool update = false) const {
  //   throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  // }

  virtual void
  getReference(Point::Current& current, unsigned int refernce_id, bool update = false) const {
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }


  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  LinearAcceleration& linear(){return _linear;} // add synchronization mechanism
  AngularAcceleration& angular(){return _angular;} // add synchronization mechanism

  const LinearAcceleration& linear() const {return _linear;} // add synchronization mechanism
  const AngularAcceleration& angular() const {return _angular;} // add synchronization mechanism

  protected:
  LinearAcceleration _linear;
  AngularAcceleration _angular;
  Interface _acceleration_interface;

};

} // namespace package
} // namespace library

#endif
