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

  SpatialVelocity(point_handling::FramePlus& frame, std::string name = "")
      : Velocity(frame, 6, name),
       _linear(frame, name), _angular(frame, name)
  {
  }

  SpatialVelocity(const Point::Current& current,
        point_handling::FramePlus& frame, std::string name = "")
      : Velocity(current, frame, name),
       _linear(frame, name), _angular(frame, name)
  {
    _current_fixed.noalias() = current;
    _linear.setFixed(current.segment<3>(3));
    _angular.setFixed(current.head<3>());
   }

  SpatialVelocity( SpatialVelocity&& other)
      : Velocity(other), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  SpatialVelocity(const SpatialVelocity& other)
      : Velocity(other), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  SpatialVelocity( SpatialVelocity&& other, point_handling::FramePlus& frame)
      : Velocity(other, frame), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  SpatialVelocity(const SpatialVelocity& other, point_handling::FramePlus& frame)
      : Velocity(other, frame), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  virtual ~SpatialVelocity() {}
  virtual SpatialVelocity* clone_impl() const {return new SpatialVelocity(*this);}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  // virtual Point::Current
  // getWorld(bool update = false) const {
  //   throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  // }

  virtual void
  getWorld(Point::Current& current, bool update = false) const {
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }
  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  // /** @brief get Position in a user-defined reference frame */
  // virtual Point::Current
  // getReference(unsigned int refernce_id, bool update = false) const{
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

  virtual const mwoibn::Matrix& getJacobian(bool update = false);
  // virtual mwoibn::Matrix getJacobian(bool update = false) const;
  virtual void getJacobian(mwoibn::Matrix& current, bool update = false) const;

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
