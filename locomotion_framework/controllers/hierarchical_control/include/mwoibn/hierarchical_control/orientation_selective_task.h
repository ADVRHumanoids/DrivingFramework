#ifndef HIERARCHICAL_CONTROL_ORIENTATION_SELECTIVE_TASK_H
#define HIERARCHICAL_CONTROL_ORIENTATION_SELECTIVE_TASK_H

#include "mwoibn/hierarchical_control/orientation_world_task.h"
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
class OrientationSelectiveTask : public OrientationWorldTask
{

public:
  /**
   * @param[in] ik the point handler mamber that defines which point is
   *controlled by this task instance it makes a local copy of a point handler to
   *prevent outside user from modifying a controlled point
   *
   */
  OrientationSelectiveTask(point_handling::OrientationsHandler ik,
                           mwoibn::VectorN selector,
                           mwoibn::robot_class::Robot& robot)
      : OrientationWorldTask(ik, robot), _selector(selector)
  {
    _resize();
    updateError();
    updateError();
    updateJacobian();
    updateJacobian();
  }

  virtual ~OrientationSelectiveTask() {}

  //! updates task error based on the current state of the robot and task
  // reference position
  //  virtual void updateError()
  //  {
  //    OrientationWorldTask::updateError();
  //    _error.noalias() = _error.cwiseProduct(_selector);
  //  }
  virtual void updateError()
  {
    _last_error.noalias() = _error;

    //  std::vector<mwoibn::Quaternion> current = _ik.getFullStatesWorld();

    int k = 0;
    for (int i = 0; i < _ik.size(); i++)
    {
      _current = _ik.getPointStateWorld(i);

      _current.ensureHemisphere(_reference[i]);

      _skew << 0, -_reference[i].z(), _reference[i].y(), _reference[i].z(), 0,
          -_reference[i].x(), -_reference[i].y(), _reference[i].x(), 0;

      _axis = _current.axis();
      _full_error.segment(k, _ik.getPointJacobianRows(i)) =
          _reference[i].w() * _axis;
      _axis = _reference[i].axis();
      _full_error.segment(k, _ik.getPointJacobianRows(i)) -=
          _current.w() * _axis;
      _full_error.segment(k, _ik.getPointJacobianRows(i)) +=
          _skew * _current.axis();

      k += _ik.getPointJacobianRows(i);

      //    _previous_state[i] = _current;
      //    std::cout << "orientation" << _error << "\n done" << std::endl;
    }

    int j = 0;

    //    std::cout << "_error" << std::endl;
    //    std::cout << _error << std::endl;
    //    std::cout << "_full_error" << std::endl;
    //    std::cout << _full_error << std::endl;
    //    std::cout << "_selector" << std::endl;
    //    std::cout << _selector << std::endl;

    for (int i = 0; i < _full_error.size(); i++)
    {

      if (_selector[i])
      {
        _error[j] = _full_error[i];
        j++;
      }
    }
  }

  //! updates task Jacobian based on the current state of the robot

  //  virtual void updateJacobian()
  //  {
  //    OrientationWorldTask::updateJacobian();

  //    for(int i = 0; i < _jacobian.rows(); i++){

  //      if (_selector[i]) continue;
  //      _jacobian.row(i).setZero();
  //    }

  //  }

  virtual void updateJacobian()
  {
    _last_jacobian.noalias() = _jacobian;
    _full_jacobian.noalias() = _ik.getFullJacobian();

    int j = 0;
    for (int i = 0; i < _full_jacobian.rows(); i++)
    {
      if (_selector[i])
      {
        _jacobian.row(j) = _full_jacobian.row(i);
        j++;
      } // it removes control over a specific angle
    }
  }

  void updateSelection(int i, double value)
  {
    if (i < _selector.size() && i >= 0)
      _selector[i] = value;

    _resize();
  }

protected:
  mwoibn::VectorN _selector, _full_error;
  mwoibn::Matrix _full_jacobian;

  void _resize()
  {
    int size = 0;
    for (int i = 0; i < _selector.size(); i++)
      if (_selector[i])
        size++;

    _full_jacobian.setZero(_jacobian.rows(), _jacobian.cols());
    _full_error.setZero(_error.size());

    _init(size, _ik.getFullJacobianCols());
  }
};
} // namespace package
} // namespace library

#endif // HIERARCHICAL_CONTROL_RIENTATION_WORLD_TASK_H
