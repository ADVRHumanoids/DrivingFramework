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
      std::find_if(_tasks_ptrs.begin(), _tasks_ptrs.end(), [&task](const std::tuple<mwoibn::VectorBool, BasicTask&, mwoibn::VectorN>& pair)
                   {
                     return &(std::get<1>(pair)) == &task;
                   });
  if (task_ptr_ == _tasks_ptrs.end())
    return 0;

  return std::get<0>(*task_ptr_).count();

}

BasicTask& getTask(int i ){
  return std::get<1>(_tasks_ptrs[i]);
}

void setWeight(double w, int task_id){
  std::get<2>(_tasks_ptrs[task_id]).setConstant(w);
}

double getWeight(int task_id){
  return std::get<2>(_tasks_ptrs[task_id])[0];
}

virtual void updateJacobian();
//! generic function to provide the same syntax for error update of all
//derived classes
virtual void updateError();
//! updates whole task in one call, calls updateError() and updateJacobin() in
//that order

virtual void update();
virtual void setVelocity(const mwoibn::VectorN& velocity){
    _my_velocity = velocity;
}


protected:
  mwoibn::VectorN _my_velocity;
  std::vector < std::tuple<mwoibn::VectorBool, BasicTask&, mwoibn::VectorN> > _tasks_ptrs;

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
