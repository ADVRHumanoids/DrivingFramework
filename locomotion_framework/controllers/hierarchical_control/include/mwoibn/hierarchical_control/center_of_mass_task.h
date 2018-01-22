#ifndef HIERARCHICAL_CONTROL_CENTER_OF_MASS_TASK_H
#define HIERARCHICAL_CONTROL_CENTER_OF_MASS_TASK_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/controller_task.h"
#include "mwoibn/robot_class/robot.h"
namespace mwoibn
{

namespace hierarchical_control
{

//! Provides a task for controlling robots center of mass
class CenterOfMassTask : public ControllerTask
{

public:
  /**
   * @param[in] ik_ptr pointer to the point handler mamber that defines which
   * point is controlled by this task instance
   */
  CenterOfMassTask(mwoibn::robot_class::Robot& robot,
                   mwoibn::VectorN reference =
                       mwoibn::VectorN::Zero(2))
      : ControllerTask(), _robot(robot), _reference(reference)
  {
    _init(2, robot.getDofs());
    _robot.centerOfMass().update(true);
    _selector = mwoibn::VectorBool::Constant(_robot.contacts().size(), true);
    _selector_dof = mwoibn::VectorBool::Constant(_robot.getDofs(), true);
//    _updateSelection();

    //    _selector.resize(robot.getDofs(), true);
    //    _updateSelection();
  }

  ~CenterOfMassTask() {}

  //! updates task error based on the current state of the robot and task
  // reference position
  virtual void updateError();
  //! updates task Jacobian based on the current state of the robot
  virtual void updateJacobian();

  //! updates task current state of the robot
  virtual void update();

  //! sets task reference
  virtual void setReference(const mwoibn::VectorN& reference)
  {
    _reference.noalias() = reference;
  }
  //! returnes task reference
  mwoibn::Matrix getReference() { return _reference; }

  //! returns chain associated with a specific contact to a Jacobian pool
  virtual void releaseContact(int i)
  {
    _selector[i] = true;
    _updateSelection();
  }

  //!
  virtual void claimContact(int i)
  {
    _selector[i] = false;
    _updateSelection();
  }

  virtual void setDofs(const mwoibn::VectorBool& dofs){
    _selector_dof = dofs;
  }

protected:
  //!pointer to the point handler mamber for point controlled by this task
  // instance
  mwoibn::robot_class::Robot& _robot;
  //! task reference position of a controlled point
  mwoibn::VectorN _reference;
  mwoibn::VectorBool _selector;
  mwoibn::VectorBool _selector_dof;

  void _updateSelection()
  {
    _selector_dof.setConstant(false);
//    std::fill(_selector_dof.begin(), _selector_dof.end(), false);
    for (int i = 0; i < _selector.size(); i++)
    {
      if (!_selector[i])
        continue;


      for (int k = 0; k < _robot.contacts().contact(i).getChain().size(); k++)
        _selector_dof[_robot.contacts().contact(i).getChain()[k]] = true;
    }

    //    for (int i = 0; i < _selector.size(); i++)
    //      std::cout << i << "\t" << _selector[i] << "\n";
    //    std::cout << std::endl;

  } // in this case only contacts that are constraint are involved in a CoM
  //  scheme
};
} // namespace package
} // namespace library

#endif // CENTER_OF_MASS_TASK_H
