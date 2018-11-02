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

  SpatialVelocity(Point::Current current,
        point_handling::FramePlus& frame, std::string name = "")
      : Velocity(current, frame, name),
       _linear(frame, name), _angular(frame, name)
  {
    _current.noalias() = current;
    _linear.setFixed(current.tail<3>());
    _angular.setFixed(current.head<3>());
   }

  SpatialVelocity(const SpatialVelocity&& other)
      : Velocity(other), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  SpatialVelocity(const SpatialVelocity& other)
      : Velocity(other), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  SpatialVelocity(const SpatialVelocity&& other, point_handling::FramePlus& frame)
      : Velocity(other, frame), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  SpatialVelocity(const SpatialVelocity& other, point_handling::FramePlus& frame)
      : Velocity(other, frame), _linear(other._linear, frame), _angular(other._angular, frame)
  {  }

  virtual ~SpatialVelocity() {}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false){} // NOT IMPLEMENTED

  virtual Point::Current
  getWorld(bool update = false) const {} // NOT IMPLEMENTED

  virtual void
  getWorld(Point::Current& current, bool update = false) const {} // NOT IMPLEMENTED
  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false){}// NOT IMPLEMENTED

  /** @brief get Position in a user-defined reference frame */
  virtual const Point::Current&
  getReference(unsigned int refernce_id, bool update = false){}// NOT IMPLEMENTED

  virtual void
  getReference(Point::Current& current, unsigned int refernce_id, bool update = false) const {}// NOT IMPLEMENTED

  virtual void setReference(const Point::Current& current,
                            unsigned int reference_id,
                            bool update = false){}// NOT IMPLEMENTED

  virtual const mwoibn::Matrix& getJacobian(bool update = false);
  virtual mwoibn::Matrix getJacobian(bool update = false) const;
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
