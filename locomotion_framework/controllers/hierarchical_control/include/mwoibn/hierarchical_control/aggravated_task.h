#ifndef __MWOIBN_HIERARCHICAL_CONTROL_AGGRAVATED_TASK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_AGGRAVATED_TASK_H

#include <rbdl/rbdl.h>
#include "mwoibn/hierarchical_control/controller_task.h"
namespace mwoibn
{

namespace hierarchical_control
{

/**
* \brief Aggravates multiple simpler tasks
*
*
*/
class AggravatedTask: public ControllerTask
{
public:
  AggravatedTask() : ControllerTask() {}

  ~AggravatedTask() {}

//  //   add task at the end of the stack
  void addTask(ControllerTask& task);

  void addTask(ControllerTask& task, mwoibn::VectorBool selector);
  // add task at the end of the stack
  void addTask(ControllerTask& task, unsigned int i);

//  // i - zero based
  void addTask(ControllerTask& task, mwoibn::VectorBool selector, unsigned int i);

  virtual void updateJacobian();
  //! generic function to provide the same syntax for error update of all
  //derived classes
  virtual void updateError();
  //! updates whole task in one call, calls updateError() and updateJacobin() in
  //that order

  virtual void update();

protected:
  std::vector < std::pair<mwoibn::VectorBool, ControllerTask&> > _tasks_ptrs;
  void _verify(ControllerTask& task, mwoibn::VectorBool selector)
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

} // namespace package
} // namespace library
#endif
