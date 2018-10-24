#ifndef __MWOIBN__POINT_HANDLING__ROTATION_H
#define __MWOIBN__POINT_HANDLING__ROTATION_H

#include "mwoibn/point_handling/base.h"

namespace mwoibn
{

namespace point_handling
{

class Rotation: public Base<mwoibn::Matrix3>
{

public:
  //typedef mwoibn::Quaternion Angular;
  typedef mwoibn::Matrix3 R;

  template<typename Body>
  Rotation(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, std::string name = "")
      : Base(body_id, model, state, 9, name)
  { _current.setIdentity();
   _temp_current.setIdentity();}

  template<typename Body>
  Rotation(Rotation::R current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        std::string name = "")
      : Base(current, body_id, model, state, 9, name)
  { }


  Rotation(const Rotation&& other)
      : Base(other)
  {  }

  Rotation(const Rotation& other)
      : Base(other)
  {  }
  virtual ~Rotation() {}

  /** @brief get Position in a world frame */
  virtual const Rotation::R&
  getWorld(bool update = false);

  virtual Rotation::R
  getWorld(bool update = false) const;
  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Rotation::R& current,
                        bool update = false);

  /** @brief get Position in a user-defined reference frame */
  virtual const Rotation::R&
  getReference(unsigned int refernce_id, bool update = false);

  virtual void setReference(const Rotation::R& current,
                            unsigned int reference_id,
                            bool update = false);

  using Base::getReference;
  using Base::setReference;

};

} // namespace package
} // namespace library

#endif
