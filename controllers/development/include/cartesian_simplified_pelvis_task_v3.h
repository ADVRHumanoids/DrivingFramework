#ifndef HIERARCHICAL_CONTROL_CATRESIAN_SIMPLIFIED_PELVIS_TASK_H
#define HIERARCHICAL_CONTROL_CATRESIAN_SIMPLIFIED_PELVIS_TASK_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/cartesian_world_task.h"
#include <rbdl/rbdl.h>

namespace mwoibn
{
namespace hierarchical_control
{

/**
 * @brief The CartesianWorldTask class Provides the inverse kinematics task
 *to control the position of a point defined in one of a robot reference frames
 *
 */
class CartesianSimplifiedPelvisTask : public CartesianWorldTask
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  CartesianSimplifiedPelvisTask(point_handling::PositionsHandler ik,
                                mwoibn::robot_class::Robot& robot)
      : CartesianWorldTask(ik), _robot(robot)
  {
    _pelvis_ptr.reset(new mwoibn::point_handling::PositionsHandler(
        "ROOT", robot, {"pelvis"}));
    _flat_model.gravity = mwoibn::Vector3(0., -9.81, 0.);
    RigidBodyDynamics::Math::Matrix3d inertia =
        RigidBodyDynamics::Math::Matrix3dZero;
    RigidBodyDynamics::Math::Vector3d com =
        RigidBodyDynamics::Math::Vector3dZero;

    RigidBodyDynamics::Body body_a(0, com, inertia);
    RigidBodyDynamics::Joint joint_a(
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 1., 0., 0.),
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 0., 1., 0.),
        RigidBodyDynamics::Math::SpatialVector(0., 0., 1., 0., 0., 0.));
    _ids.push_back(_flat_model.AddBody(
        0, RigidBodyDynamics::Math::Xtrans(
               RigidBodyDynamics::Math::Vector3d(0., 0., 0.)),
        joint_a, body_a));

    RigidBodyDynamics::Body body_b(0, com, inertia);
    RigidBodyDynamics::Joint joint_b(
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 0., 0., 1.),
        RigidBodyDynamics::Math::SpatialVector(0., 1., 0., 0., 0., 0.),
        RigidBodyDynamics::Math::SpatialVector(1., 0., 0., 0., 0., 0.));
    _ids.push_back(_flat_model.AddBody(
        _ids.back(), RigidBodyDynamics::Math::Xtrans(
                         RigidBodyDynamics::Math::Vector3d(0., 0., 0.)),
        joint_b, body_b));
    _init(_ik.size() * 2, _ik.getFullJacobianCols());
    _state = mwoibn::VectorN::Zero(6);
    //    _point_flat = mwoibn::VectorN::Zero(_ik.size()*3);
    mwoibn::VectorN temp_reference = _reference;
    _reference = mwoibn::VectorN::Zero(_ik.size() * 2);

    updateState();
    _height.resize(_ik.size());

    for (int i = 0; i < _ik.size(); i++){
      setReferenceWorld(i, temp_reference.segment(i * 3, 3), false);
      _height[i] = temp_reference[i*3+2];
    }
  }

  virtual ~CartesianSimplifiedPelvisTask() {}

  //! updates task error based on the current state of the robot and task
  // reference position
  virtual void updateError()
  {
    _last_error = _error; // save previous state
    updateState();

    for (int i = 0; i < _ik.size(); i++)
    {
      _point_flat = getPointStateReference(i);
      _error.segment(2 * i, 2) =
          _reference.segment(2 * i, 2) - _point_flat.head(2);

//      std::cout << i << std::endl;
//      std::cout << "point flat" << std::endl;
//      std::cout << _point_flat << std::endl;

    }


  }

  void updateState()
  {
    mwoibn::Vector3 temp;
    // update state

    temp = _pelvis_ptr->getPointStateWorld(0);
    _state[0] = temp[0];
    _state[1] = temp[1];
    _state[3] = temp[2];

    temp = _pelvis_ptr->point(0)
               .getRotationWorld(
                    _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION))
               .transpose()
               .eulerAngles(2, 1, 0);

//    ensure the angles are in the correct ranges (-pi:pi, -pi/2:pi/2, -pi:pi)
//    std::cout << "update state 0" << temp << std::endl;
//    std::cout << "original state" << _state << std::endl;

    temp[0] -= 6.28318531 * std::floor((temp[0] + 3.14159265) / 6.28318531);
    temp[1] -= 6.28318531 * std::floor((temp[1] + 3.14159265) / 6.28318531);
    temp[2] -= 6.28318531 * std::floor((temp[2] + 3.14159265) / 6.28318531);

    if(std::fabs(temp[0]-_state[2]) > 1.57079633 && std::fabs(temp[0]-_state[2]) < 4.71238898 ){
//      std::cout << "change" << std::endl;
      temp[0] -= 3.14159265;
      temp[1] = -temp[1];
      temp[2] -= 3.14159265;
    }
    else
//      std::cout << "it's OK" << std::endl;

    temp[0] -= 6.28318531 * std::floor((temp[0] + 3.14159265) / 6.28318531);
    temp[1] -= 6.28318531 * std::floor((temp[1] + 3.14159265) / 6.28318531);
    temp[2] -= 6.28318531 * std::floor((temp[2] + 3.14159265) / 6.28318531);

    _state[2] = temp[0];
    _state[4] = temp[1];
    _state[5] = temp[2];

//    std::cout << "update state 1" << temp << std::endl;

    RigidBodyDynamics::UpdateKinematicsCustom(_flat_model, &_state, NULL, NULL);
  }

  virtual void updateJacobian()
  {
    _last_jacobian = _jacobian;

    mwoibn::Matrix new_jacobian = mwoibn::Matrix::Zero(
        _ik.getFullJacobianRows(), _ik.getFullJacobianCols());

    mwoibn::Matrix new_state =
        mwoibn::Matrix::Zero(_ik.size() * 2, 6);

    _ik.getFullJacobian(new_jacobian);
//    std::cout << "new jacobian" << std::endl;
//    std::cout << new_jacobian << std::endl;

    for (int i = 0; i < _ik.size(); i++)
    {

      _jacobian.block(2 * i, 0, 2, _robot.getDofs()) =
          new_jacobian.block(3 * i, 0, 2, _robot.getDofs());

      mwoibn::Matrix new_point_state = mwoibn::Matrix::Zero(3, 6);

      _point_flat = RigidBodyDynamics::CalcBaseToBodyCoordinates(
          _flat_model, _state, _ids[1], _ik.getPointStateWorld(i), false);

      RigidBodyDynamics::CalcPointJacobian(_flat_model, _state, _ids[1],
                                           _point_flat, new_point_state, false);

      new_state.block(2 * i, 0, 2, 6) = new_point_state.topRows(2);
    }

//    std::cout << "new_state\t" << new_state << std::endl;

    mwoibn::Matrix jacobian =
        (mwoibn::eigen_utils::pseudoInverse<mwoibn::Matrix>(
             new_state.leftCols(6), 1e-8) *
         _jacobian.leftCols(6));

    new_state.leftCols(3) = mwoibn::Matrix::Zero(new_state.rows(), 3);
    mwoibn::Matrix rotation(2, 2);
    rotation << std::cos(_state[2]), std::sin(_state[2]), -std::sin(_state[2]),
        std::cos(_state[2]);

    _jacobian.leftCols(6) = (new_state)*jacobian;

    for (int i = 0; i < _ik.size(); i++)
      _jacobian.block(2 * i, 0, 2, _robot.getDofs()) =
          rotation * _jacobian.block(2 * i, 0, 2, _robot.getDofs());

    _jacobian = -_jacobian;

//    std::cout << "jacobian" << std::endl;
//    std::cout << _jacobian << std::endl;
  }

  //  virtual void resetReference() { _reference = _ik.getFullStateReference();
  //  } // it will have to be redefined

  using CartesianWorldTask::getReference;
  using CartesianWorldTask::setReference;

  virtual mwoibn::VectorN getReference(int i) const
  {
    return _reference.segment(i * 2, 2);
  }

  virtual void setReference(int i, mwoibn::VectorN reference)
  {
    _reference.segment(i * 2, 2) = reference;
  }

  virtual void setReferenceWorld(int i, mwoibn::Vector3 reference, bool update)
  {
    if (update)
      updateState();

    _reference.segment(i * 2, 2) =
        RigidBodyDynamics::CalcBaseToBodyCoordinates(
            _flat_model, _state, _ids[0], reference, false).head(2);
  }

  virtual mwoibn::Vector3
  getReferenceWorld(int i, bool update) // it can have update as it uses RBDL
                                        // call and cannot be constant anyway
  {
    if (update)
      updateState();

    mwoibn::Vector3 reference;
    reference.head(2) = _reference.segment(i * 2, 2);
    reference[2] = _height[i];
    return RigidBodyDynamics::CalcBodyToBaseCoordinates(
        _flat_model, _state, _ids[0], reference, false);
  }

  virtual mwoibn::Vector3 getPointStateReference(int i)
  {
    return RigidBodyDynamics::CalcBaseToBodyCoordinates(
        _flat_model, _state, _ids[0], _ik.getPointStateWorld(i), false);
  }

  mwoibn::VectorN getState() const { return _state; }

protected:
  mwoibn::VectorN _state;
  std::unique_ptr<mwoibn::point_handling::PositionsHandler> _pelvis_ptr;
  mwoibn::robot_class::Robot& _robot;
  RigidBodyDynamics::Model _flat_model;
  mwoibn::Vector3 _point_flat;
  std::vector<int> _ids;
  mwoibn::VectorN _height;
};
} // namespace package
} // namespace library
#endif
