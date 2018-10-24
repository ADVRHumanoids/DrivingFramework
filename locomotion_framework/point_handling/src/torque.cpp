#include "mwoibn/point_handling/torque.h"

namespace mwoibn
{
namespace point_handling
{


  const Point::Current&
  Torque::getWorld(bool update){
        _temp_current = _rotation.getWorld(update)*(_current + _position.getWorld(false).head<3>().cross(_force.getFixed().head<3>()));
  }

  Point::Current
  Torque::getWorld(bool update) const {
        return _rotation.getWorld(update)*(_current + _position.getWorld(false).head<3>().cross(_force.getFixed().head<3>()));
  }

  void Torque::getWorld(Point::Current& angular, bool update) const {

        angular = _rotation.getWorld(update)*(_current + _position.getWorld(false).head<3>().cross(_force.getFixed().head<3>()));
        return;
  }


  void Torque::setWorld(const Point::Current& angular, bool update){

        _current = _rotation.getWorld(update).transpose()*angular - _position.getWorld(false).head<3>().cross(_force.getFixed().head<3>());

  }

  const Point::Current&
  Torque::getReference(unsigned int reference_id, bool update){

            _temp_current = _rotation.getReference(reference_id, update)*(_current + _position.getReference(reference_id, false).head<3>().cross(_force.getFixed().head<3>()));
  }

  void Torque::setReference(const Point::Current& angular, unsigned int reference_id, bool update){

            _current = _rotation.getReference(reference_id, update).transpose()*angular - _position.getReference(reference_id, false).head<3>().cross(_force.getFixed().head<3>());
  }


} // namespace package
} // namespace library
