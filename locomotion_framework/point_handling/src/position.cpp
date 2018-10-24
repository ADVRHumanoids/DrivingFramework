#include "mwoibn/point_handling/position.h"

namespace mwoibn
{
namespace point_handling
{


const Point::Current&
Position::getWorld(bool update)
{
  _temp_current = CalcBodyToBaseCoordinates(_model, _state.get(), _body_id,
                                             _current, update);
  return _temp_current;
}

Point::Current
Position::getWorld(bool update) const{
  return CalcBodyToBaseCoordinates(_model, _state.get(), _body_id,
                                             _current, update);
}

void Position::setWorld(const Point::Current& position,
                             bool update)
{
  _current = CalcBaseToBodyCoordinates(_model, _state.get(), _body_id,
                                        position, update);
}

const Point::Current&
Position::getReference(unsigned int refernce_id, bool update)
{
  _temp_current =
      CalcBaseToBodyCoordinates(_model, _state.get(), refernce_id,
                                getWorld(), update);
  return _temp_current;
}

void Position::setReference(const Point::Current& position,
                                 unsigned int reference_id,
                                 bool update)
{

  Point::Current world_position = CalcBodyToBaseCoordinates(
      _model, _state.get(), reference_id, position, update);

  setWorld(world_position);
}

// const mwoibn::Matrix&
// Position::getJacobian(bool update)
// {
//
//   _J_part.setZero();
//   CalcPointJacobian(_model, _state.get(), _body_id, _current, _J_part,
//                     update);
//
//   return _J_part;
// }
//
// mwoibn::Matrix
// Position::getJacobian(bool update) const{
//
//   mwoibn::Matrix J = mwoibn::Matrix::Zero(_J_part.rows(), _J_part.cols());
//
//   CalcPointJacobian(_model, _state.get(), _body_id, _current, J,
//                     update);
//
//   return J;
// }

} // namespace package
} // namespace library
