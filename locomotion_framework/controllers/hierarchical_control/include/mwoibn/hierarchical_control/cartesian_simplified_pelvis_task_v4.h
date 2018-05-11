#ifndef HIERARCHICAL_CONTROL_CATRESIAN_SIMPLIFIED_PELVIS_TASK_4_H
#define HIERARCHICAL_CONTROL_CATRESIAN_SIMPLIFIED_PELVIS_TASK_4_H

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
class CartesianFlatReferenceTaskV4 : public ContactPoint2DRbdlTask
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  CartesianFlatReferenceTaskV4(point_handling::PositionsHandler ik,
                                mwoibn::robot_class::Robot& robot)
      : ContactPoint2DRbdlTask(ik, robot)
  {
    init();
  }

  virtual ~CartesianFlatReferenceTaskV4() {}

  virtual void updateError()
  {
    _last_error.noalias() = _error; // save previous state
    //updateState(); // this will be updated by steering call

    for (int i = 0; i < _ik.size(); i++)
    {

      _rotation << std::cos( _state[2]), -std::sin( _state[2]),
          std::sin( _state[2]), std::cos( _state[2]);

      _full_error.segment(2 * i, 2).noalias() = _robot.centerOfMass().get().head(2) + _rotation*_reference.segment(2 * i, 2) - _ik.getPointStateWorld(i).head(2);

      _error[i] = _directions[2*i] * _full_error[2*i] + _directions[2*i + 1] * _full_error[2*i+1];
    }

  }

  virtual void updateJacobian()
  {
    _last_jacobian.noalias() = _jacobian;

    for (int i = 0; i < _ik.size(); i++)
    {
      _jacobian.row(i).noalias() = _directions.segment<2>(2*i).transpose() *( _robot.centerOfMass().getJacobian().topRows<2>() - _ik.getFullPointJacobian(i).topRows<2>());
    }

  }

  virtual const mwoibn::Vector3& getPointStateReference(int i)
  {
    _rotation << std::cos(_state[2]), std::sin(_state[2]), 0,
        -std::sin(_state[2]), std::cos(_state[2]), 0, 0, 0, 1;

    _track_point = _ik.getPointStateWorld(i);
    _track_point.head<2>() -= _robot.centerOfMass().get().head<2>();

    _point.noalias() = _rotation * _track_point;

    return _point;
      }

protected:
  mwoibn::VectorN _wheels;
  mwoibn::Vector3 _point_flat, _track_point;

};
} // namespace package
} // namespace library
#endif
