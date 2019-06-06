#include "mwoibn/point_handling/force.h"

namespace mwoibn
{
namespace point_handling
{


  const Point::Current&
  Force::getWorld(bool update)
  {
    _temp_world = frame.rotation().getWorld(update)*_current_fixed;

    return _temp_world;
  }

  void Force::getWorld(Point::Current& current, bool update) const
  {
    current.head<3>() = frame.rotation().getWorld(update)*_current_fixed;
  }

  void Force::setWorld( const Point::Current& linear,
                               bool update)
  {
    _current_fixed = frame.rotation().getWorld(update).transpose()*linear;
  }

  // Point::Current
  // Force::getWorld(bool update) const{
  //   return frame.rotation().getWorld(update)*_current_fixed;
  // }


  // Point::Current
  // Force::getReference( unsigned int refernce_id, bool update) const
  // {
  //   return frame.rotation().getReference(refernce_id, update)*_current_fixed;
  // }


  void Force::getReference(Point::Current& current, unsigned int refernce_id, bool update) const
  {
    current.head<3>() =  frame.rotation().getReference(refernce_id, update)*_current_fixed;

  }

  void Force::setReference( const Point::Current& position,
                                   unsigned int reference_id,
                                   bool update)
  {
    _current_fixed =  frame.rotation().getReference(reference_id, update).transpose()*_current_fixed;
  }


} // namespace package
} // namespace library
