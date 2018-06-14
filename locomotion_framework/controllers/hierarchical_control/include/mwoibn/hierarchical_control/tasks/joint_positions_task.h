#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_JOINT_POSITIONS_TASK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_JOINT_POSITIONS_TASK_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_class/robot.h"
#include <rbdl/rbdl.h>

namespace mwoibn
{

namespace hierarchical_control
{
namespace tasks
{

//! Provides the inverse kinematics task to control the position of a point
// defined in one of a robot reference frames
/**
 *
 * \todo provide the option of modyfing only the part of error (i.e. only one
 *point)
 * \todo add option of adding/removing points
 *
 */
class JointPosition : public BasicTask
{

public:
  /**
   * @param[in] ik_ptr pointer to the point handler mamber that defines which
   * point is controlled by this task instance
   */
  JointPosition(robot_class::Robot& robot,
                    mwoibn::VectorN selector)
      : BasicTask(), _robot(robot)
  {
    _reference = mwoibn::VectorN::Zero(robot.getDofs());

    try
    {
      _init(selector);
    }
    catch (const std::invalid_argument& e)
    {
      throw;
    }
  }

  JointPosition(robot_class::Robot& robot,
                    mwoibn::VectorN selector,
                    mwoibn::VectorN reference)
      : BasicTask(), _robot(robot)
  {
    if (reference.size() != _robot.getDofs())
    {
      std::stringstream errMsg;
      errMsg << "Wrong size of reference expected '" << _robot.getDofs()
             << ", received " << reference.size();
      throw(std::invalid_argument(errMsg.str().c_str()));
    }

    _reference = reference;

    try
    {
      _init(selector);
    }
    catch (const std::invalid_argument& e)
    {
      throw;
    }
  }
  ~JointPosition() {}

  //! updates task error based on the current state of the robot and task
  // reference position
  virtual void updateError()
  {
    _last_error = _error;
    _error = _reference - _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);
    _error = _error.cwiseProduct(_selector);
  }
  //! updates task Jacobian based on the current state of the robot
  virtual void updateJacobian() {}
  //! sets task reference getting only actuated dofs
  virtual void setReference(mwoibn::VectorN reference)
  {
    _reference = reference;
  }

  //! returnes task reference
  mwoibn::VectorN getReference() { return _reference; }

  int getDofs() { return _reference.size(); }

protected:
  //! task reference position of a controlled point
  mwoibn::VectorN _reference;
  robot_class::Robot& _robot;

  void _init(mwoibn::VectorN selector)
  {
    if (selector.size() != _robot.getDofs())
    {
      std::stringstream errMsg;
      errMsg << "Wrong size of selector expected '" << _robot.getDofs()
             << ", received " << selector.size();
      throw(std::invalid_argument(errMsg.str().c_str()));
    }

    _selector = selector;

    BasicTask::_init(_reference.size(), _robot.getDofs());
    _jacobian = -mwoibn::Matrix::Identity(_reference.size(),
                                                             _reference.size());
    for(int i = 0; i < _jacobian.cols(); i++)
      _jacobian.col(i) = _jacobian.col(i).cwiseProduct(_selector);

//    std::cout << "_jacobian" << std::endl;
//    std::cout << _jacobian << std::endl;
    BasicTask::updateJacobian(getJacobian());
  }

  /** Defines the importance of this task for each of the rbdl dofs, tha values
   * should be between <0,1>, size of the vector has to be of the size of robot
   * dofs */
  mwoibn::VectorN _selector;
};
}
} // namespace package
} // namespace library
#endif
