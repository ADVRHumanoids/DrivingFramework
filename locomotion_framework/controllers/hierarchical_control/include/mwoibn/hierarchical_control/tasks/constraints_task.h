#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONSTRAINTS_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONSTRAINTS_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_class/robot.h"
namespace mwoibn {

namespace hierarchical_control
{
namespace tasks
{

//! Provides a task for controlling robots center of mass
class Constraints : public BasicTask
{

public:

/**
 * @param[in] ik_ptr pointer to the point handler mamber that defines which point is controlled by this task instance
 */
Constraints(mwoibn::robot_class::Robot& robot, std::vector<std::string> active_groups = {})
        : BasicTask(), _robot(robot)
{
        if(active_groups.empty()){
          _init(robot.contacts().jacobianRows(), robot.getDofs());
          _selector.assign(robot.contacts().size(), true); // on init assume all constacts should be considered in a task
          std::cout << "no active groups" << std::endl;
        }
        else{
          _selector.assign(robot.contacts().size(), false); // on init assume all constacts should be considered in a task

          int size = 0;
          for(auto& group: active_groups){
            for(auto& contact: _robot.contacts().group(group)) {
              size += contact->jacobianSize();
              _selector[_robot.contacts().getId(contact->getName())] = true;
              // std::cout << "active groups\t" << std::count(_selector.begin(), _selector.end(), true)  << std::endl;
              // std::cout << contact->getName() << "\t" << _robot.contacts().getId(contact->getName()) << std::endl;
            }
          }
          _init(size, robot.getDofs());

        }

        update();
}

virtual ~Constraints() {
}

//! updates task error based on the current state of the robot and task reference position
virtual void updateError();
//! updates task Jacobian based on the current state of the robot
virtual void updateJacobian();

//! returns chain associated with a specific contact to a Jacobian pool
virtual void releaseContact(int i){
        _selector[i] = true;
}
//!
virtual void claimContact(int i){
        _selector[i] = false;
}
virtual void changeContacts(std::vector<int> dofs){
        dof_selector = dofs;
}

protected:
//!pointer to the point handler mamber for point controlled by this task instance
mwoibn::robot_class::Robot& _robot;
//! keeps an information which contacts should be off by higher level scheme
// mwoibn::VectorBool _selector;
std::vector<bool> _selector;
std::vector<int> dof_selector;

};
}
} // namespace package
} // namespace library
#endif
