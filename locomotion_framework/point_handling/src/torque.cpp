#include "mwoibn/point_handling/torque.h"

namespace mwoibn
{
namespace point_handling
{


  const Point::Current&
  Torque::getWorld(bool update){
        _temp_world = frame.rotation().getWorld(update)*(_current_fixed + frame.position.getWorld(false).head<3>().cross(force.getFixed().head<3>()));
  }

  Point::Current
  Torque::getWorld(bool update) const {
        return frame.rotation().getWorld(update)*(_current_fixed + frame.position.getWorld(false).head<3>().cross(force.getFixed().head<3>()));
  }

  void Torque::getWorld(Point::Current& angular, bool update) const {

        angular.head<3>() = frame.rotation().getWorld(update)*(_current_fixed + frame.position.getWorld(false).head<3>().cross(force.getFixed().head<3>()));
        return;
  }


  void Torque::setWorld(const Point::Current& angular, bool update){

        _current_fixed = frame.rotation().getWorld(update).transpose()*angular - frame.position.getWorld(false).head<3>().cross(force.getFixed().head<3>());

  }

  Point::Current
  Torque::getReference(unsigned int reference_id, bool update) const {

            return frame.rotation().getReference(reference_id, update)*(_current_fixed + frame.position.getReference(reference_id, false).head<3>().cross(force.getFixed().head<3>()));
  }

  void Torque::getReference(Point::Current& current, unsigned int reference_id, bool update) const {
        current.head<3>()= frame.rotation().getReference(reference_id, update)*(_current_fixed + frame.position.getReference(reference_id, false).head<3>().cross(force.getFixed().head<3>()));
  }

  void Torque::setReference(const Point::Current& angular, unsigned int reference_id, bool update){

            _current_fixed = frame.rotation().getReference(reference_id, update).transpose()*angular - frame.position.getReference(reference_id, false).head<3>().cross(force.getFixed().head<3>());
  }


} // namespace package
} // namespace library
