#ifndef HIERARCHICAL_CONTROL_CONSTRAINTS_TASK_H
#define HIERARCHICAL_CONTROL_CONSTRAINTS_TASK_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/controller_task.h"
#include "mwoibn/robot_class/robot.h"
namespace mwoibn{

namespace hierarchical_control
{

//! Provides a task for controlling robots center of mass
class ConstraintsTask : public ControllerTask
{

public:

  /**
   * @param[in] ik_ptr pointer to the point handler mamber that defines which point is controlled by this task instance
   */
  ConstraintsTask(mwoibn::robot_class::Robot& robot)
      : ControllerTask(), _robot(robot)
  {
    _init(robot.contacts().jacobianRows(), robot.getDofs());
    _selector = mwoibn::VectorBool::Constant(robot.contacts().size(), true); // on init assume all constacts should be considered in a task

    update();
  }

  ~ConstraintsTask() {}

  //! updates task error based on the current state of the robot and task reference position
  virtual void updateError();
  //! updates task Jacobian based on the current state of the robot
  virtual void updateJacobian();

  //! returns chain associated with a specific contact to a Jacobian pool
  virtual void releaseContact(int i){_selector[i] = true;}
  //!
  virtual void claimContact(int i){_selector[i] = false;}
  virtual void changeContacts(std::vector<int> dofs){dof_selector = dofs;}

protected:
  //!pointer to the point handler mamber for point controlled by this task instance
  mwoibn::robot_class::Robot& _robot;
  //! keeps an information which contacts should be off by higher level scheme
  mwoibn::VectorBool _selector;
  std::vector<int> dof_selector;

//  mwoibn::VectorN _reference;
};
} // namespace package
} // namespace library

#endif // HIERARCHICAL_CONTROL_CONSTRAINTS_TASK_H
