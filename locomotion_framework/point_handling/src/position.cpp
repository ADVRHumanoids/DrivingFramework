#include "mwoibn/point_handling/position.h"

namespace mwoibn
{
namespace point_handling
{


const Point::Current&
Position::getWorld(bool update)
{
  _temp_world = CalcBodyToBaseCoordinates(_model, _state.position.get(), _body_id,
                                             _current_fixed, update);
  return _temp_world;
}

// Point::Current
// Position::getWorld(bool update) const{
//   return CalcBodyToBaseCoordinates(_model, _state.position.get(), _body_id,
//                                              _current_fixed, update);
// }

void Position::getWorld(Point::Current& current, bool update) const{
  current.head<3>() = CalcBodyToBaseCoordinates(_model, _state.position.get(), _body_id,
                                                _current_fixed, update);
}

void Position::getWorld(mwoibn::Vector3& current, bool update) const{
  current.head<3>() = CalcBodyToBaseCoordinates(_model, _state.position.get(), _body_id,
                                                _current_fixed, update);
}

void Position::setWorld(const Point::Current& position,
                             bool update)
{
  _current_fixed = CalcBaseToBodyCoordinates(_model, _state.position.get(), _body_id,
                                        position, update);
}

const Point::Current&
Position::getReference(unsigned int refernce_id, bool update)
{
  _temp_world.head<3>() = CalcBaseToBodyCoordinates(_model, _state.position.get(), refernce_id,
                                getWorld(), update);
  return _temp_world;
}


void
Position::getReference(Point::Current& current, unsigned int refernce_id, bool update) const
{
  getWorld(current, update);

  current.head<3>() =  CalcBaseToBodyCoordinates(_model, _state.position.get(), refernce_id,
                                current, false);
}


void Position::setReference(const Point::Current& position,
                                 unsigned int reference_id,
                                 bool update)
{

  Point::Current world_position = CalcBodyToBaseCoordinates(
      _model, _state.position.get(), reference_id, position, update);

  setWorld(world_position);
}

// const mwoibn::Matrix&
// Position::getJacobian(bool update)
// {
//
//   _J_part.setZero();
//   CalcPointJacobian(_model, _state.position.get(), _body_id, _current_fixed, _J_part,
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
//   CalcPointJacobian(_model, _state.position.get(), _body_id, _current_fixed, J,
//                     update);
//
//   return J;
// }

} // namespace package
} // namespace library
