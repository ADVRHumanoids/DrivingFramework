#ifndef __MWOIBN__POINT_HANDLING__LINEAR_VELOCITY_H
#define __MWOIBN__POINT_HANDLING__LINEAR_VELOCITY_H

#include "mwoibn/point_handling/velocity.h"
#include "mwoibn/point_handling/frame_plus.h"

namespace mwoibn
{

namespace point_handling
{

class LinearVelocity: public Velocity
{

public:

  LinearVelocity(point_handling::FramePlus& frame, std::string name = "")
      : Velocity(frame, 3, name)
  {  }

  LinearVelocity(Point::Current current,
        point_handling::FramePlus& frame, std::string name = "")
      : Velocity(current, frame, name)
  { }

  LinearVelocity( LinearVelocity&& other)
      : Velocity(other)
  {  }

  LinearVelocity(const LinearVelocity& other)
      : Velocity(other)
  {  }

  LinearVelocity( LinearVelocity&& other, point_handling::FramePlus& frame)
      : Velocity(other, frame)
  {  }

  LinearVelocity(const LinearVelocity& other, point_handling::FramePlus& frame)
      : Velocity(other, frame)
  {  }

  virtual ~LinearVelocity() {}

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getWorld(bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  virtual Point::Current
  getWorld(bool update = false) const {
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }


  virtual void
  getWorld(Point::Current& current, bool update = false) const {
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Point::Current& current,
                        bool update = false){
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

  /** @brief get Position in a user-defined reference frame */
  virtual Point::Current
  getReference(unsigned int refernce_id, bool update = false) const{
    throw mwoibn::std_utils::notImplemented(__PRETTY_FUNCTION__);
  }

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
  virtual mwoibn::Matrix getJacobian(bool update = false) const;
  virtual void getJacobian(mwoibn::Matrix& current, bool update = false) const;

};

} // namespace package
} // namespace library

#endif
