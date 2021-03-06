#include "mwoibn/point_handling/wrench.h"

namespace mwoibn
{
namespace point_handling
{

  const Point::Current&
  Wrench::getWorld(bool update){

    _get(_temp_world, force.getWorld(update), torque.getWorld(false));

    return _temp_world;
  }

  void
  Wrench::getWorld(Point::Current& current, bool update) const{
    torque.getWorld(current, false);
    current.segment<3>(3) = current.head<3>();
    force.getWorld(current,update);
  }

  // Point::Current
  // Wrench::getWorld(bool update) const{
  //
  //   Point::Current current(size());
  //   _get(current, force.getWorld(update), torque.getWorld(false));
  //
  //   return current;
  // }

  void Wrench::setFixed(const mwoibn::Vector6& current){

      _temp.head<3>() = current.head<3>();
      torque.setFixed(_temp);

      _temp.head<3>() = current.segment<3>(3);
      force.setFixed(_temp);

      synch();
    }

  void Wrench::setFixed(const Point::Current& current){

    _temp.head<3>() = current.head<3>();
    torque.setFixed(_temp);

    _temp.head<3>() = current.segment<3>(3);
    force.setFixed(_temp);

    synch();
  }


  /** @brief set new tracked point giving data in a world frame*/
  void Wrench::setWorld(const Point::Current& linear,
                        bool update){

    _temp.head<3>() = linear.head<3>();
    torque.setWorld(_temp, update);

    _temp.head<3>() = linear.segment<3>(3);
    force.setWorld(_temp, update);

    synch();
  }

  // /** @brief get Position in a user-defined reference frame */
  // Point::Current
  // Wrench::getReference(unsigned int refernce_id, bool update) const{
  //
  //     Point::Current reference_(size());
  //     _get(reference_, force.getReference(refernce_id, update), torque.getReference(refernce_id, false));
  //
  //     return reference_;
  // }

  void
  Wrench::getReference(Point::Current& current, unsigned int refernce_id, bool update) const{
    torque.getReference(current, false);
    current.segment<3>(3) = current.head<3>();
    force.getReference(current,update);

  }

  void Wrench::setReference(const Point::Current& position,
                            unsigned int reference_id,
                            bool update){
      force.setReference(position.segment<3>(3), reference_id, update);
      torque.setReference(position.head<3>(), reference_id, false);

      synch();

  }

  void Wrench::synch(){
    _get(_current_fixed, force.getFixed(), torque.getFixed());
  }

  void Wrench::_get(Point::Current& current, const Point::Current& force, const Point::Current& torque) const {
    current.head<3>() = torque;
    current.segment<3>(3) = force;
  }



} // namespace package
} // namespace library
