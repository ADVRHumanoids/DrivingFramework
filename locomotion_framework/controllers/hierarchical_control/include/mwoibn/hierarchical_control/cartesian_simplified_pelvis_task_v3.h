#ifndef HIERARCHICAL_CONTROL_CATRESIAN_SIMPLIFIED_PELVIS_TASK_H
#define HIERARCHICAL_CONTROL_CATRESIAN_SIMPLIFIED_PELVIS_TASK_H

#include "mwoibn/hierarchical_control/contact_point_2D_rbdl_task.h"

namespace mwoibn
{
namespace hierarchical_control
{

/**
 * @brief The CartesianWorldTask class Provides the inverse kinematics task
 *to control the position of a point defined in one of a robot reference frames
 *
 */
class CartesianSimplifiedPelvisTask : public ContactPoint2DRbdlTask
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
      : ContactPoint2DRbdlTask(ik, robot)
  {


    _jacobian_3D.setZero(_ik.getPointJacobianRows(0),
                         _ik.getFullPointJacobianCols(0));

    _jacobian_flat_3D.setZero(3, 6);
    _jacobian_2D.setZero(2 * ik.size(), _robot.getDofs());
    _jacobian_2D_bis.setZero(2, _robot.getDofs());
    _jacobian_flat_2D.setZero(_ik.size() * 2, 6);
    _inverser.init(_jacobian_flat_2D, 1e-6);
    _jacobian6.setZero(6, 6);
    _point2D.setZero(2);

    init();

  }

  virtual ~CartesianSimplifiedPelvisTask() {}

  //! updates task error based on the current state of the robot and task
  // reference position
  virtual void updateError()
  {
    _last_error.noalias() = _error; // save previous state
    //updateState(); // this will be updated by steering call

    for (int i = 0; i < _ik.size(); i++)
    {
      _point_flat = getPointStateReference(i);

      _rotation << std::cos( _state[2]), -std::sin( _state[2]),
          std::sin( _state[2]), std::cos( _state[2]);

      _full_error.segment(2 * i, 2).noalias() =
          _reference.segment(2 * i, 2) - _point_flat.head(2);

      _point2D.noalias() = _rotation * _full_error.segment(2 * i, 2);

      _error[i] = _directions[2*i] * _point2D[0] + _directions[2*i + 1] * _point2D[1];

    }

  }

  virtual void updateJacobian()
  {
    _last_jacobian.noalias() = _jacobian;

    for (int i = 0; i < _ik.size(); i++)
    {
      _jacobian_3D.noalias() = _ik.getFullPointJacobian(i);

      _jacobian_2D.block(2 * i, 0, 2, _robot.getDofs()) =
          _jacobian_3D.topRows<2>();

      _point_flat = RigidBodyDynamics::CalcBaseToBodyCoordinates(
          _flat_model, _state, _ids[1], _ik.getPointStateWorld(i), false);

      _jacobian_flat_3D.setZero();
      RigidBodyDynamics::CalcPointJacobian(
          _flat_model, _state, _ids[1], _point_flat, _jacobian_flat_3D, false);

      _jacobian_flat_2D.block<2, 6>(2 * i, 0) = _jacobian_flat_3D.topRows(2);
    }

    _inverser.compute(_jacobian_flat_2D);

    _jacobian6.noalias() = _inverser.get() * _jacobian_2D.leftCols(6);

    _jacobian_flat_2D.leftCols(3).setZero();

    _jacobian_2D.leftCols(6).noalias() = (_jacobian_flat_2D)*_jacobian6;

    for (int i = 0; i < _ik.size(); i++)
    {
      _jacobian.row(i).noalias() = - _directions.segment<2>(2*i).transpose() * _jacobian_2D.block(2 * i, 0, 2, _robot.getDofs());
    }

  }

  virtual const mwoibn::Vector3& getPointStateReference(int i)
  {
    _point =  RigidBodyDynamics::CalcBaseToBodyCoordinates(
          _flat_model, _state, _ids[0], _ik.getPointStateWorld(i), false);

    return _point;
  }


protected:
  mwoibn::VectorN _wheels, _point2D;

  mwoibn::Vector3 _point_flat;

  mwoibn::Matrix _jacobian_3D, _jacobian_flat_3D, _jacobian_flat_2D,
      _jacobian6, _jacobian_2D, _jacobian_2D_bis;
  mwoibn::PseudoInverse _inverser;
};
} // namespace package
} // namespace library
#endif
