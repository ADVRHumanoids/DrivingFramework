#ifndef CONTACT_POINT_3D_RBDL_TASK_H
#define CONTACT_POINT_3D_RBDL_TASK_H

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
class ContactPoint3DRbdlTask : public ContactPointRbdlTask
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  ContactPoint3DRbdlTask(point_handling::PositionsHandler ik,
                         mwoibn::robot_class::Robot& robot)
      : ContactPointRbdlTask(ik, robot)
  {

    _init(3 * _ik.size(), _ik.getFullJacobianCols());

    _reference.setZero(_ik.size() * 3);

    _rotation.setZero(3, 3);

    _full_error.setZero(ik.size() * 3);

    _temp_jacobian.setZero(3, _robot.getDofs());
    _point_jacobian.setZero(3, _robot.getDofs());

    _selector = mwoibn::VectorBool::Constant(
        _robot.contacts().size(),
        true); // on init assume all constacts should be considered in a task

    mwoibn::Axis axis; // this should go to the config files
    axis << 0, 0, 1;
    _axes.push_back(axis);
    axis << 0, 0, -1;
    _axes.push_back(axis);
    axis << 0, 0, 1;
    _axes.push_back(axis);
    axis << 0, 0, -1;
    _axes.push_back(axis);

    _axes_world = _axes;
    _x_world = _axes;

    _ground_normal << 0, 0, 1;
  }

  virtual ~ContactPoint3DRbdlTask() {}

  virtual void init()
  {

    updateState();

    for (int i = 0; i < _ik.size(); i++)
    {
      _reference.segment<3>(3 * i) = getPointStateReference(i);
    }
  }

  void updateState()
  {
    // update state

    ContactPointRbdlTask::updateState();
    for (int i = 0; i < _ik.size(); i++)
    {
      _ground_normal << 0, 0, 1; // HARDCODED
      _axes_world[i] =
          _wheels_ptr->point(i)
              .getRotationWorld(
                   _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION)) *
          _axes[i]; // z axis, for our kinematics wheel axis in general

      _x_world[i] = _axes_world[i].cross(_ground_normal); //?
      _x_world[i].normalize();
    }

    _updateTransform();
  }


  virtual void updateError()
  {
    _last_error.noalias() = _error; // save previous state

    for (int i = 0; i < _ik.size(); i++)
    {

      _full_error.segment<3>(3*i) = _referencePoint(i) - _contactPoint(i);

      if (_selector[i])
      {
        _error[3 * i] = _x_world[i][0] * _full_error[3 * i] +
                        _x_world[i][1] * _full_error[3 * i + 1];
        _error[3 * i + 1] = 0;
        _error[3 * i + 2] = 0;
      }
      else
      {
        _error.segment<3>(3 * i) = _full_error.segment<3>(
            3 * i); // here I should change to keep the first task the same
      }
    }
  }


  virtual void updateJacobian()
  {
    _last_jacobian.noalias() = _jacobian;

    for (int i = 0; i < _ik.size(); i++)
    {

      _point_jacobian.noalias() = _referenceJacobian(i);
      _point_jacobian -= _contactJacobian(i);

      if (_selector[i])
      {
        _jacobian.row(3 * i).noalias() =
            _x_world[i].transpose() * _point_jacobian;
        _jacobian.row(3 * i + 1).setZero();
        _jacobian.row(3 * i + 2).setZero();
      }
      else
      {
        _jacobian.block(3 * i, 0, 3, _robot.getDofs()) = _point_jacobian;
      }
    }
  }

  using CartesianWorldTask::getReference;
  using CartesianWorldTask::setReference;

  virtual mwoibn::VectorN getReference(int i) const
  {
    return _reference.segment<3>(i * 3);
  }

  virtual void setReference(int i, const mwoibn::Vector3& reference)
  {
    _reference.segment(i * 3, 3) = reference;
  }


  virtual void setReferenceWorld(int i, const mwoibn::Vector3& reference,
                                 bool update)
  {

    if (update)
      updateState();

    _reference.segment<3>(i*3) = _worldToBase(reference);

  }

  virtual mwoibn::Vector3
  getReferenceWorld(int i) // it can have update as it uses RBDL
                                        // call and cannot be constant anyway
  {

    mwoibn::Vector3 reference;
    reference = _reference.segment(i * 3, 3);

    return _baseToWorld(reference);
  }


  virtual const mwoibn::Vector3& getPointStateReference(int i)
  {
    _point.noalias() = _worldToBase(_contactPoint(i));
    return _point;
  }

  virtual const mwoibn::Vector3& getReferenceError(int i)
  {
    _point.noalias() = _getTransform().transpose() * (_full_error.segment<3>(3 * i));
    return _point;
  }

  virtual void releaseContact(int i) { _selector[i] = true; }
  virtual void claimContact(int i) { _selector[i] = false; }

protected:
  std::vector<mwoibn::Axis> _axes, _axes_world, _x_world;
  mwoibn::VectorBool _selector;
  mwoibn::Axis _ground_normal;
  mwoibn::VectorN _full_error;
  mwoibn::Matrix3 _rotation;
  mwoibn::Matrix _temp_jacobian, _point_jacobian;
  mwoibn::Vector3 _point;

  virtual void _updateTransform() = 0;

  virtual mwoibn::Vector3 _referencePoint(int i)
  {

    _temp_point = _reference.segment<3>(3 * i);

    return _baseToWorld(_temp_point);
  }

  virtual mwoibn::Vector3 _contactPoint(int i) = 0;

  virtual const mwoibn::Matrix& _referenceJacobian(int i) = 0;
  virtual const mwoibn::Matrix& _contactJacobian(int i) = 0;

  virtual mwoibn::Vector3 _worldToBase(mwoibn::Vector3 point) = 0;
  virtual mwoibn::Vector3 _baseToWorld(mwoibn::Vector3 point) = 0;


};
} // namespace package
} // namespace library
#endif
