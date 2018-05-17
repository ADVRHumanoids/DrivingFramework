#include "mwoibn/point_handling/point.h"

namespace mwoibn
{
namespace point_handling
{

const Point::Position&
Point::getPositionWorld(const mwoibn::VectorN& joint_positions, bool update)
{
  _temp_position = CalcBodyToBaseCoordinates(_model, joint_positions, _body_id,
                                             _position, update);
  return _temp_position;
}

const mwoibn::Vector7&
Point::getFullStateWorld(const mwoibn::VectorN& joint_positions, bool update)
{

  getPositionWorld(joint_positions, update);
  getOrientationWorld(joint_positions, update);
  toFullState(_temp_full, _temp_position, _temp_orientation);

  return _temp_full;
}

void Point::setPositionWorld(const Point::Position position,
                             const mwoibn::VectorN& joint_positions,
                             bool update)
{
  _position = CalcBaseToBodyCoordinates(_model, joint_positions, _body_id,
                                        position, update);
}

void Point::fromFullState(const mwoibn::Vector7& full_state,
                          Point::Position& position,
                          Point::Orientation& orientation)
{
  position = full_state.head(3);
  orientation.x() = full_state[3];
  orientation.y() = full_state[4];
  orientation.z() = full_state[5];
  orientation.w() = full_state[6];
}
void Point::toFullState(mwoibn::Vector7& full_state,
                        const Point::Position& position,
                        const Point::Orientation& orientation)
{
  full_state.head(3) = position;
  full_state[3] = orientation.x();
  full_state[4] = orientation.y();
  full_state[5] = orientation.z();
  full_state[6] = orientation.w();
}
void Point::setFullStateWorld(const mwoibn::Vector7 state,
                              const mwoibn::VectorN& joint_positions,
                              bool update)
{
  fromFullState(state, _temp_position, _temp_orientation);
  setPositionWorld(_temp_position, joint_positions, update);
  setOrientationWorld(_temp_orientation, joint_positions, false);
}

const Point::Position&
Point::getPositionReference(unsigned int refernce_id,
                            const mwoibn::VectorN& joint_positions, bool update)
{
  _temp_position =
      CalcBaseToBodyCoordinates(_model, joint_positions, refernce_id,
                                getPositionWorld(joint_positions), update);
  return _temp_position;
}

const mwoibn::Vector7&
Point::getFullStateReference(unsigned int refernce_id,
                             const mwoibn::VectorN& joint_positions,
                             bool update)
{
  _temp_position = getPositionReference(refernce_id, joint_positions, update);
  _temp_orientation =
      getOrientationReference(refernce_id, joint_positions, false);
  toFullState(_temp_full, _temp_position, _temp_orientation);

  return _temp_full;
}

const mwoibn::Vector7&
Point::getFullStateReference(std::string reference_name,
                             const mwoibn::VectorN& joint_positions,
                             bool update)
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

  return getFullStateReference(body_id, joint_positions, update);
}

void Point::setPositionReference(const Point::Position position,
                                 unsigned int reference_id,
                                 const mwoibn::VectorN& joint_positions,
                                 bool update)
{

  Point::Position world_position = CalcBodyToBaseCoordinates(
      _model, joint_positions, reference_id, position, update);

  setPositionWorld(world_position, joint_positions);
}

void Point::setFullStateReference(const mwoibn::Vector7 state,
                                  unsigned int reference_id,
                                  const mwoibn::VectorN& joint_positions,
                                  bool update)
{
  fromFullState(state, _temp_position, _temp_orientation);
  setPositionReference(_temp_position, reference_id, joint_positions, update);
  setOrientationReference(_temp_orientation, reference_id, joint_positions,
                          false);
}

void Point::setFullStateReference(const mwoibn::Vector7 state,
                                  std::string reference_name,
                                  const mwoibn::VectorN& joint_positions,
                                  bool update)
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

  return setFullStateReference(state, body_id, joint_positions, update);
}

const mwoibn::Matrix&
Point::getPositionJacobian(const mwoibn::VectorN& joint_positions, bool update)
{

  _J_part.setZero();
  CalcPointJacobian(_model, joint_positions, _body_id, _position, _J_part,
                    update);

  return _J_part;
}

mwoibn::Matrix
Point::getPositionJacobian(const mwoibn::VectorN& joint_positions,
                    bool update) const{

  mwoibn::Matrix J = mwoibn::Matrix::Zero(_J_part.rows(), _J_part.cols());

  CalcPointJacobian(_model, joint_positions, _body_id, _position, J,
                    update);

  return J;
}

/** @bried returnes full jacobian of a point **/
const mwoibn::Matrix&
Point::getOrientationJacobian(const mwoibn::VectorN& joint_positions,
                              bool update)
{

  _J.setZero();
  CalcPointJacobian6D(_model, joint_positions, _body_id, _position, _J, update);

  _J_part.noalias() = _J.topRows<3>();

  return _J_part;
}

mwoibn::Matrix Point::getOrientationJacobian(const mwoibn::VectorN& joint_positions,
                       bool update) const{
  mwoibn::Matrix J = mwoibn::Matrix::Zero(_J.rows(), _J.cols());

  CalcPointJacobian6D(_model, joint_positions, _body_id, _position, J, update);
  mwoibn::Matrix J_part = J.topRows<3>();
  return J_part;
}

/** @bried returnes full jacobian of a point **/
const mwoibn::Matrix&
Point::getFullJacobian(const mwoibn::VectorN& joint_positions, bool update)
{

  _J.setZero();
  CalcPointJacobian6D(_model, joint_positions, _body_id, _position, _J, update);
  return _J;
}

const Point::Position&
Point::getPositionReference(std::string reference_name,
                            const mwoibn::VectorN& joint_positions, bool update)
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
  return getPositionReference(body_id, joint_positions, update);
}

void Point::setPositionReference(const Point::Position position,
                                 std::string reference_name,
                                 const mwoibn::VectorN& joint_positions,
                                 bool update)
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

  setPositionReference(position, body_id, joint_positions, update);
}

const Point::Rotation&
Point::getRotationWorld(const mwoibn::VectorN& joint_positions, bool update)
{

  _temp_rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
                       _model, joint_positions, _body_id, update) *
                   getRotationFixed().transpose();

  return _temp_rotation;
}

Point::Rotation Point::getRotationWorld(const mwoibn::VectorN& joint_positions,
                                        bool update) const
{  
  return RigidBodyDynamics::CalcBodyWorldOrientation(_model, joint_positions,
                                                     _body_id, update) *
         getRotationFixed().transpose();
}

const Point::Rotation&
Point::getRotationReference(unsigned int reference_id,
                            const mwoibn::VectorN& joint_positions, bool update)
{

  _temp_rotation =
      RigidBodyDynamics::CalcBodyWorldOrientation(
          _model, joint_positions, reference_id, update).transpose() *
      getRotationWorld(joint_positions);

  return _temp_rotation;
}

const Point::Rotation&
Point::getRotationReference(std::string reference_name,
                            const mwoibn::VectorN& joint_positions, bool update)
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
  return getRotationReference(body_id, joint_positions, update);
}

Point::Rotation Point::getRotationFixed() const
{

  return _orientation.toMatrix();
}

const Point::Rotation& Point::getRotationFixed()
{
  _temp_rotation = _orientation.toMatrix();
  return _temp_rotation;
}

void Point::setRotationFixed(const Point::Rotation rotation)
{
  _orientation = mwoibn::Quaternion::fromMatrix(rotation);
}

void Point::setRotationWorld(const Point::Rotation rotation,
                             const mwoibn::VectorN& joint_positions,
                             bool update)
{

  _orientation = Point::Orientation();
  Point::Rotation fixed =
      getRotationWorld(joint_positions, update).transpose() * rotation;
  setRotationFixed(fixed);
}

void Point::setRotationReference(const Point::Rotation rotation,
                                 unsigned int reference_id,
                                 const mwoibn::VectorN& joint_positions,
                                 bool update)
{

  _orientation = Point::Orientation();
  Point::Rotation fixed =
      getRotationReference(reference_id, joint_positions, update).transpose() *
      rotation;

  setRotationFixed(fixed);
}

void Point::setRotationReference(const Point::Rotation rotation,
                                 std::string reference_name,
                                 const mwoibn::VectorN& joint_positions,
                                 bool update)
{

  _orientation = Point::Orientation();
  Point::Rotation fixed = getRotationReference(reference_name, joint_positions,
                                               update).transpose() *
                          rotation;

  setRotationFixed(fixed);
}

} // namespace package
} // namespace library
