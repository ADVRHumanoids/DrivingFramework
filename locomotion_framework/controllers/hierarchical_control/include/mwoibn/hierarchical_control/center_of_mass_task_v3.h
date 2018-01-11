#ifndef HIERARCHICAL_CONTROL_CENTER_OF_MASS_TASK_3_H
#define HIERARCHICAL_CONTROL_CENTER_OF_MASS_TASK_3_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/controller_task.h"
#include "mwoibn/robot_class/robot.h"
namespace mwoibn
{

namespace hierarchical_control
{

//! Provides a task for controlling robots center of mass
class CenterOfMassTask3 : public ControllerTask
{

public:
  /**
   * @param[in] ik_ptr pointer to the point handler mamber that defines which
   * point is controlled by this task instance
   */
  CenterOfMassTask3(mwoibn::robot_class::Robot& robot, mwoibn::VectorBool which,
                   mwoibn::VectorN reference =
                       mwoibn::VectorN::Zero(2))
      : ControllerTask(), _robot(robot), _reference(reference), _which(which)
  {
    int size;

    for(int i = 0; i < _which.size(); i++)
        if(_which[i]) size++;

    _init(2, size);
    _robot.centerOfMass().update(true);
  }

  ~CenterOfMassTask3() {}

  //! updates task error based on the current state of the robot and task
  // reference position
  virtual void updateError();
  //! updates task Jacobian based on the current state of the robot
  virtual void updateJacobian();

  //! updates task current state of the robot
  virtual void update();

  //! sets task reference
  virtual void setReference(mwoibn::VectorN reference)
  {
    _reference = reference;
  }
  //! returnes task reference
  mwoibn::Matrix getReference() { return _reference; }

protected:
  //!pointer to the point handler mamber for point controlled by this task
  // instance
  mwoibn::robot_class::Robot& _robot;
  //! task reference position of a controlled point
  mwoibn::VectorN _reference;
  mwoibn::VectorBool _which;

};
} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_TASK_H
