#ifndef CONTACT_POINT_2D_RBDL_TASK_H
#define CONTACT_POINT_2D_RBDL_TASK_H

#include "mwoibn/hierarchical_control/contact_point_rbdl_task.h"

namespace mwoibn
{
namespace hierarchical_control
{

/**
 * @brief The CartesianWorldTask class Provides the inverse kinematics task
 *to control the position of a point defined in one of a robot reference frames
 *
 */
class ContactPoint2DRbdlTask : public ContactPointRbdlTask
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  ContactPoint2DRbdlTask(point_handling::PositionsHandler ik,
                                mwoibn::robot_class::Robot& robot)
      : ContactPointRbdlTask(ik, robot)
  {
    _init(_ik.size(), _ik.getFullJacobianCols());

    _reference.setZero(_ik.size() * 2);
    _directions.setZero(_ik.size() * 2);
    _height.setZero(_ik.size());

    _rotation.setZero(2, 2);

    _full_error.setZero(ik.size() * 2);
  }

  virtual ~ContactPoint2DRbdlTask() {}

  virtual void init()
  {

    updateState();

    for (int i = 0; i < _ik.size(); i++)
    {
      _temp_point = _ik.getPointStateWorld(i);
      setReferenceWorld(i, _temp_point, false);
      _height[i] = _temp_point[2];
    }
  }

  virtual void updateState()
  {
    // update state
    ContactPointRbdlTask::updateState();

    for (int i = 0; i <  _ik.size(); i++)
    {
      mwoibn::Vector3 axis, direction;
      axis << 0, 0, 1;
      direction = _wheels_ptr->point(i)
                           .getRotationWorld(_robot.state.get(
                               mwoibn::robot_class::INTERFACE::POSITION))
                           .col(2); // z axis, for our kinematics wheel axis in general

      direction = direction.cross(axis); //?
      direction.normalize();

      _directions.segment<2>(2*i) = direction.head(2);
    }
  }

  using CartesianWorldTask::getReference;
  using CartesianWorldTask::setReference;

  virtual mwoibn::VectorN getReference(int i) const
  {
    return _reference.segment(i * 2, 2);
  }

  virtual void setReference(int i, const mwoibn::Vector3& reference)
  {
    _reference.segment(i * 2, 2) = reference.head<2>();
  }


  virtual void setReferenceWorld(int i, const mwoibn::Vector3& reference,
                                 bool update)
  {
    if (update)
      updateState();

    _reference.segment(i * 2, 2) =
        RigidBodyDynamics::CalcBaseToBodyCoordinates(
            _flat_model, _state, _ids[0], reference, false).head(2);
  }

  virtual mwoibn::Vector3
  getReferenceWorld(int i) // it can have update as it uses RBDL
                                        // call and cannot be constant anyway
  {

    mwoibn::Vector3 reference;
    reference.head(2) = _reference.segment(i * 2, 2);
    reference[2] = _height[i];
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(
        _flat_model, _state, _ids[0], reference, false);
  }

  virtual const mwoibn::Vector3& getReferenceError(int i)
  {
    _rotation << std::cos(_state[2]), std::sin(_state[2]), 0,
        -std::sin(_state[2]), std::cos(_state[2]), 0, 0, 0, 1;
    _temp_point.head<2>() = _full_error.segment<2>(2 * i);
    _temp_point[2] = _height[i];
    _point.noalias() = _rotation * _temp_point;
    return _point;
  }

  virtual const mwoibn::Vector3& getPointStateReference(int i) = 0;


protected:
  mwoibn::VectorN _height, _full_error, _directions;

  mwoibn::Matrix  _rotation;
  mwoibn::Vector3 _point;


};
} // namespace package
} // namespace library
#endif
