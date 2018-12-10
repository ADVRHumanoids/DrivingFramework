#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_AGGRAVATED_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_AGGRAVATED_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
namespace mwoibn
{

namespace hierarchical_control
{
namespace tasks {
/**
 * \brief Aggravates multiple simpler tasks
 *
 *
 */
class Aggravated : public BasicTask
{
public:
Aggravated() : BasicTask() {
}

virtual ~Aggravated() {
}

//  //   add task at the end of the stack
void addTask(BasicTask& task);

void addTask(BasicTask& task, mwoibn::VectorBool selector);
// add task at the end of the stack
void addTask(BasicTask& task, unsigned int i);

//  // i - zero based
void addTask(BasicTask& task, mwoibn::VectorBool selector, unsigned int i);

int taskSize(const BasicTask& task){
  auto task_ptr_ =
      std::find_if(_tasks_ptrs.begin(), _tasks_ptrs.end(), [&task](const std::pair<mwoibn::VectorBool, BasicTask&>& pair)
                   {
                     return &(pair.second) == &task;
                   });
  if (task_ptr_ == _tasks_ptrs.end())
    return 0;

  return task_ptr_->first.count();

}

virtual void updateJacobian();
//! generic function to provide the same syntax for error update of all
//derived classes
virtual void updateError();
//! updates whole task in one call, calls updateError() and updateJacobin() in
//that order

virtual void update();

protected:
std::vector < std::pair<mwoibn::VectorBool, BasicTask&> > _tasks_ptrs;
void _verify(BasicTask& task, mwoibn::VectorBool selector)
{
        if (task.getTaskSize() != selector.size())
                throw std::invalid_argument(std::string(
                                                    "Aggravated Task: task and selector sizes do not match."));

        if(_tasks_ptrs.size() == 0) return;

        if (task.getTaskDofs() != getTaskDofs())
                throw std::invalid_argument(
                              std::string("Aggravated Task: couldn't append a task, state sizes do "
                                          "not match."));
}
};
}
} // namespace package
} // namespace library
#endif
