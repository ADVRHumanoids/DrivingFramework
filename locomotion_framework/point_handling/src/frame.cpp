#include "mwoibn/point_handling/frame.h"

namespace mwoibn
{
namespace point_handling
{


const mwoibn::Vector7&
Frame::getFullStateWorld(bool update)
{

  getLinearWorld(update);
  getOrientationWorld(update);
  toFullState(_temp_full, _position.getFixed(), _orientation.getFixed());

  return _temp_full;
}

void Frame::fromFullState(const mwoibn::Vector7& full_state,
                          Point::Current& linear,
                          Orientation::O& orientation)
{
  linear.head<3>() = full_state.head<3>();
  orientation.x() = full_state[3];
  orientation.y() = full_state[4];
  orientation.z() = full_state[5];
  orientation.w() = full_state[6];
}
void Frame::toFullState(mwoibn::Vector7& full_state,
                        const Point::Current& linear,
                        const Orientation::O& orientation)
{
  full_state.head<3>() = linear.head<3>();
  full_state[3] = orientation.x();
  full_state[4] = orientation.y();
  full_state[5] = orientation.z();
  full_state[6] = orientation.w();
}
void Frame::setFullStateWorld(const mwoibn::Vector7 state, bool update)
{
  fromFullState(state, _pos, _quat);
  setLinearWorld(_pos, update);
  setOrientationWorld(_quat, false);
}

const mwoibn::Vector7&
Frame::getFullStateReference(unsigned int refernce_id, bool update)
{
  _pos = getLinearReference(refernce_id, update);
  _quat = getOrientationReference(refernce_id, false);
  toFullState(_temp_full, _pos, _quat);

  return _temp_full;
}

const mwoibn::Vector7&
Frame::getFullStateReference(std::string reference_name, bool update)
{
  unsigned int body_id;
  try
  {
    body_id = _checkBody(reference_name);
  }
  catch (const std::invalid_argument& e)
  {
    throw;
  }

  return getFullStateReference(body_id, update);
}


const mwoibn::Matrix&
Frame::getPositionJacobian(bool update)
{
  return _velocity.linear().getJacobian();
}

mwoibn::Matrix
Frame::getPositionJacobian(bool update) const{
  return _velocity.linear().getJacobian();
}

/** @bried returnes full jacobian of a point **/
const mwoibn::Matrix&
Frame::getOrientationJacobian(bool update)
{
  return _velocity.angular().getJacobian();
}

mwoibn::Matrix Frame::getOrientationJacobian(bool update) const{
  return _velocity.angular().getJacobian();
}

/** @bried returnes full jacobian of a point **/
const mwoibn::Matrix&
Frame::getFullJacobian(bool update)
{
  return _velocity.getJacobian();
}

Rotation::R Frame::getRotationFixed() const
{

  return _orientation.rotation().getFixed();
}

const Rotation::R& Frame::getRotationFixed()
{
  return _orientation.rotation().getFixed();
}

void Frame::setRotationFixed(const Rotation::R rotation)
{
  _orientation.rotation().setFixed(rotation);
  _orientation.synch();
}

void Frame::setRotationWorld(const Rotation::R rotation, bool update)
{

  _orientation.rotation().setWorld(rotation, update);
  _orientation.synch();
}





} // namespace package
} // namespace library
