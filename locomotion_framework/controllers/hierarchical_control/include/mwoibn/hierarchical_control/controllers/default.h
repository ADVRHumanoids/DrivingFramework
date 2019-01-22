#ifndef __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_DEFAULT_H
#define __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_DEFAULT_H

//#include <rbdl/rbdl.h>
#include "mwoibn/hierarchical_control/controllers/basic.h"
#include <boost/bimap.hpp>

// temporary libraries
#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>
/**
 * \todo change the package name to hierarchical_control
 * \todo add to the naming convention do not name the package and internal class
 *******************************the same
 *
 */
namespace mwoibn {
namespace hierarchical_control {
namespace controllers {
//! Implementation of a hierarchical controller
/**
 * \todo add a reference to the paper about the implemented controller
 * \todo add task names for convenience
 * \todo add some utility functions, so that the user can get some information
 *******************************about tasks (combined with the name utility)
 * \todo add Default cartesian position task
 * \todo add a constructor taking the tasks (with variable number of arguments)
 * \todo **extract the continous adjustement to the task change as an inheriting
 *******************************class**
 *
 */
class Default : public Basic
{
public:
Default() : Basic() {
}

//Default(const Default& other) : Basic(other), _gains(other._gains), _errors(other._errors), _inversers_ptrs(other._inversers_ptrs), _tasks(other._tasks), _P(other._P), _dofs(other._dofs)  {
//}

Default(const Default& other) : Basic(other), _P(other._P), _dofs(other._dofs), _errors(other._errors)  {

  for(int i =0; i < other._tasks.size(); i++){
    addTask(other._tasks[i], other._gains[i], other._inversers_ptrs[i]->damping(0));
  }

}


virtual ~Default() {
}
virtual void init(){
}
virtual int size() {
        return _tasks.size();
}

/**
 * \brief The addition of a task to the stuck
 *
 * @param[in] i if defined the controller will add the task in a specified
 *******************************place task[i] pushing back lower priorty tasks, otherwise a task is added at
 *******************************the and of a stack.
 *
 * \note for the size-variable tasks the gain needs to be initialized by the
 *******************************vector with maximum number of gains
 * \todo improve handling of size-variable tasks (adjust the gains size)
 */
virtual void addTask(tasks::BasicTask& new_task, mwoibn::VectorN gain,
                     double damping = 1e-8);
virtual void addTask(tasks::BasicTask& new_task, double gain,
                     double damping = 1e-8);
virtual void addTask(tasks::BasicTask& new_task, unsigned int i, mwoibn::VectorN gain,
                     double damping = 1e-8);
virtual void addTask(tasks::BasicTask& new_task, unsigned int i, double gain,
                     double damping = 1e-8);



//! Removes a task from the stack
virtual void removeTask(unsigned int i);


/**
 * \see Default#compute()
 *
 */
virtual const mwoibn::VectorN& update();
//! computes the controll law without updating the controllers states
/**
 * \see Default#updateController()
 *
 */
virtual void compute();

/** @brief Allows to modify controller gains online **/
bool updateGain(int i, mwoibn::VectorN gain)
{
        _gains.at(i).noalias() = gain;
        return true;
}
bool updateGain(int i, double gain)
{
        _gains.at(i).noalias() =
                gain * mwoibn::VectorN::Ones(_gains.at(i).size());
        return true;
}
//  bool updateDamping(int i, double damp)
//  {
//    _damping.at(i) = damp;
//    return true;
//  }

protected:
/** \brief Comprises pointers to all of the tasks,
 * in the priority order.
 *
 * The lower position in the stack the higher priority (i.e tasks[0] is more
 *******************************important than tasks[1])
 */
//! keeps tasks gains
std::vector<mwoibn::VectorN> _gains;
std::vector<mwoibn::VectorN> _errors;
std::vector<std::unique_ptr<mwoibn::Projection> > _inversers_ptrs;
virtual void _updateTask(int i, mwoibn::hierarchical_control::tasks::BasicTask& task);
std::vector<std::reference_wrapper<tasks::BasicTask> > _tasks;

virtual void _init();
mwoibn::Matrix _P;
double _dofs;
};
} // namespace package
} // namespace library
}

#endif
