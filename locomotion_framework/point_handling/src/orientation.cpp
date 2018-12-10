#include "mwoibn/point_handling/orientation.h"

namespace mwoibn
{
namespace point_handling
{


  /** @brief get orientation in a world frame as a quaternion
   */
  const Orientation::O&
  Orientation::getWorld(bool update)
  {
    _temp_world = _set(_rotation.getWorld(update));
    return _temp_world;
  }
  /** @brief get orientation in a world frame as a quaternion
   */
  Orientation::O Orientation::getWorld(bool update) const
  {
    return _set(_rotation.getWorld(update));
  }

  /** @brief set new tracked point giving data in a world frame
   */
  void Orientation::setWorld(const Orientation::O& quat, bool update)
  {
    _rotation.setWorld(quat.toMatrix(), update);
    _current_fixed = _set(_rotation.getFixed());

  }

  /** @brief get Position in a user-defined reference frame
   */
  Orientation::O
  Orientation::getReference(unsigned int reference_id, bool update) const
  {
    return _set(_rotation.getReference(reference_id, update));
  }


  /** @brief set new tracked point giving data in a user-defined reference
   * frame

   */
  void Orientation::setReference(const Orientation::O& quat,
                               unsigned int reference_id,
                               bool update)
  {

    _rotation.setReference(quat.toMatrix(), reference_id, update);
    _current_fixed = _set(_rotation.getFixed());
  }




} // namespace package
} // namespace library
