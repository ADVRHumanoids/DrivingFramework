#include "mwoibn/point_handling/force.h"

namespace mwoibn
{
namespace point_handling
{


  const Point::Current&
  Force::getWorld(bool update)
  {
    _temp_current = frame.rotation().getWorld(update)*_current;

    return _temp_current;
  }


  void Force::setWorld( const Point::Current& linear,
                               bool update)
  {
    _current = frame.rotation().getWorld(update).transpose()*linear;
  }

  Point::Current
  Force::getWorld(bool update) const{
    return frame.rotation().getWorld(update)*_current;
  }


  const Point::Current&
  Force::getReference( unsigned int refernce_id, bool update)
  {
    _temp_current =  frame.rotation().getReference(refernce_id, update)*_current;
    return _temp_current;
  }

  void Force::setReference( const Point::Current& position,
                                   unsigned int reference_id,
                                   bool update)
  {
    _current =  frame.rotation().getReference(reference_id, update).transpose()*_current;
  }


} // namespace package
} // namespace library
