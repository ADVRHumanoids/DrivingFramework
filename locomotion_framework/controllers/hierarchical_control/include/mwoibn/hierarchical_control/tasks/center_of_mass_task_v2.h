#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CENTER_OF_MASS_2_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CENTER_OF_MASS_2_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_class/robot.h"
namespace mwoibn
{

namespace hierarchical_control
{
namespace tasks
{

//! Provides a task for controlling robots center of mass
class CenterOfMass2 : public BasicTask
{

public:
  /**
   * @param[in] ik_ptr pointer to the point handler mamber that defines which
   * point is controlled by this task instance
   */
  CenterOfMass2(mwoibn::robot_class::Robot& robot, mwoibn::VectorInt map,
                   mwoibn::VectorN reference =
                       mwoibn::VectorN::Zero(2))
      : BasicTask(), _robot(robot), _reference(reference), _map(map)
  {
    _init(2, map.size());
    _robot.centerOfMass().update(true);
    _selector = mwoibn::VectorBool::Constant(_robot.contacts().size(), true);
    _selector_dof = mwoibn::VectorBool::Constant(_robot.getDofs(), true);
//    _updateSelection();

    //    _selector.resize(robot.getDofs(), true);
    //    _updateSelection();
  }

  ~CenterOfMass2() {}

  //! updates task error based on the current state of the robot and task
  // reference position
  virtual void updateError();
  //! updates task Jacobian based on the current state of the robot
  virtual void updateJacobian();

  //! updates task current state of the robot
  virtual void update();

  //! sets task reference
  template<typename Vector>
  void setReference(const Vector& reference)
  {
    _reference[0] = reference[0];
    _reference[1] = reference[1];
  }

  //! returnes task reference
  const mwoibn::VectorN& getReference() { return _reference; }

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

protected:
  //!pointer to the point handler mamber for point controlled by this task
  // instance
  mwoibn::robot_class::Robot& _robot;
  //! task reference position of a controlled point
  mwoibn::VectorN _reference;
  mwoibn::VectorBool _selector;
  mwoibn::VectorBool _selector_dof;
  mwoibn::VectorInt _map;

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
}
} // namespace package
} // namespace library
#endif
