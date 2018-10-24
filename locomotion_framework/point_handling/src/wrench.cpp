#include "mwoibn/point_handling/wrench.h"

namespace mwoibn
{
namespace point_handling
{

  const Point::Current&
  Wrench::getWorld(bool update){

    _get(_temp_current, _force.getWorld(update), _torque.getWorld(false));

    return _temp_current;
  }


  Point::Current
  Wrench::getWorld(bool update) const{

    Point::Current current(size());
    _get(current, _force.getWorld(update), _torque.getWorld(false));

    return current;
  }

  void Wrench::setFixed(const Point::Current& current){

    _force.setFixed(current.tail<3>());
    _torque.setFixed(current.head<3>());

    synch();
  }


  /** @brief set new tracked point giving data in a world frame*/
  void Wrench::setWorld(const Point::Current& linear,
                        bool update){
    _force.setWorld(linear.tail<3>(), update);
    _torque.setWorld(linear.head<3>(), false);

    synch();
  }

  /** @brief get Position in a user-defined reference frame */
  const Point::Current&
  Wrench::getReference(unsigned int refernce_id, bool update){

        _get(_temp_current, _force.getReference(refernce_id, update), _torque.getReference(refernce_id, false));

        return _temp_current;
  }

  void Wrench::setReference(const Point::Current& position,
                            unsigned int reference_id,
                            bool update){
      _force.setReference(position.tail<3>(), reference_id, update);
      _torque.setReference(position.head<3>(), reference_id, false);

      synch();

  }

  void Wrench::synch(){
    _get(_current, _force.getFixed(), _torque.getFixed());
  }

  void Wrench::_get(Point::Current& current, const Point::Current& force, const Point::Current& torque) const {
    current.head<3>() = torque;
    current.tail<3>() = force;
  }



} // namespace package
} // namespace library
