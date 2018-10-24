#ifndef __MWOIBN__POINT_HANDLING__ORIENTATION_H
#define __MWOIBN__POINT_HANDLING__ORIENTATION_H

#include "mwoibn/point_handling/base.h"
#include "mwoibn/point_handling/rotation.h"

namespace mwoibn
{

namespace point_handling
{

class Orientation: public Base<mwoibn::Quaternion>
{

public:
  typedef mwoibn::Quaternion O;

  template<typename Body>
  Orientation(Body body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, std::string name = "")
      : Base(body_id, model, state, 4, name), _rotation(body_id, model, state, name)
  {
  }

  template<typename Body>
  Orientation(Orientation::O current, Body body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        std::string name = "")
      : Base(body_id, model, state, 4, name), _rotation(body_id, model, state, name)
  {
  }

  Orientation(const Orientation&& other)
      : Base(other), _rotation(other._rotation)
  {  }

  Orientation(const Orientation& other)
      : Base(other), _rotation(other._rotation)
  {  }

  virtual ~Orientation() {}
  /** @brief get Position in a point fixed frame*/

  void setFixed(const Orientation::O& current){ _current = current;
                                      _rotation.setFixed(_current.toMatrix()); }

  /** @brief get Position in a world frame */
  virtual const Orientation::O&
  getWorld(bool update = false);

  virtual Orientation::O
  getWorld(bool update = false) const;
  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Orientation::O& current,
                        bool update = false);

  /** @brief get Position in a user-defined reference frame */
  virtual const Orientation::O&
  getReference(unsigned int refernce_id, bool update = false);

  using Base::getReference;
  using Base::setReference;

  virtual void setReference(const Orientation::O& current,
                            unsigned int reference_id,
                            bool update = false);

  point_handling::Rotation& rotation(){return _rotation;}
  const point_handling::Rotation& rotation() const{return _rotation;}

  void synch(){ _current = _set(_rotation.getFixed());}

  protected:
  point_handling::Rotation _rotation;

  Orientation::O _set(Rotation::R rot) const{
      return Orientation::O::fromMatrix(rot);
  }
};

} // namespace package
} // namespace library

#endif
