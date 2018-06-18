#ifndef __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_ACTIONS_H
#define __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_ACTIONS_H


#include "mwoibn/hierarchical_control/controllers/basic.h"

#include "mwoibn/hierarchical_control/actions/basic_action.h"
#include "mwoibn/hierarchical_control/actions/task.h"

#include "mwoibn/hierarchical_control/actions/idle.h"
#include "mwoibn/hierarchical_control/actions/snap.h"
#include "mwoibn/hierarchical_control/actions/replace.h"
#include "mwoibn/hierarchical_control/controllers/memory_manager.h"


namespace mwoibn {
namespace hierarchical_control {
namespace controllers {


class Actions : public Basic
{
public:
Actions(double dt, unsigned int dofs);
virtual ~Actions() {
}
virtual void init(){
        // _construct();
}

// adds task to the controller and the active stack
virtual void addTask(tasks::BasicTask& new_task, mwoibn::VectorN gain,
                     double damping = 1e-8);
virtual void addTask(tasks::BasicTask& new_task, double gain,
                     double damping = 1e-8);
virtual void addTask(tasks::BasicTask& new_task, unsigned int i, mwoibn::VectorN gain,
                     double damping = 1e-8);
virtual void addTask(tasks::BasicTask& new_task, unsigned int i, double gain,
                     double damping = 1e-8);

// add task to the controller without activating it
virtual void idleTask(tasks::BasicTask& new_task, mwoibn::VectorN gain,
                      double damping = 1e-8);
virtual void idleTask(tasks::BasicTask& new_task, double gain,
                      double damping = 1e-8);

virtual actions::Replace* replace(tasks::BasicTask& task, double mu);

//! Removes a task from the stack
virtual void removeTask(unsigned int i);
virtual void removeTask(tasks::BasicTask& task);


virtual const mwoibn::VectorN& update();
//! computes the controll law without updating the controllers states
/**
 * \see Basic#updateController()
 *
 */
virtual void compute();

/** @brief Allows to modify controller gains online **/
bool updateGain(tasks::BasicTask& task, const mwoibn::VectorN& gain);
bool updateGain(tasks::BasicTask& task, double gain);

// returns current active stack
virtual int size(){
        return _active_stack.size();
}
protected:
/** \brief Comprises pointers to all of the tasks,
 * in the priority order.
 *
 * The lower position in the stack the higher priority (i.e tasks[0] is more
 * important than tasks[1])
 */
protected:
// virtual void _construct() = 0;
void _addTask(tasks::BasicTask& new_task, double gain, double damping);
void _addTask(tasks::BasicTask& new_task, mwoibn::VectorN& gain, double damping);
bool _setIdle(tasks::BasicTask& new_task);

std::vector<actions::Task* > _active_stack;

std::vector<actions::Snap > _snap;
std::vector<actions::Replace > _replace;
std::vector<actions::Secondary > _secondary;

std::vector<std::unique_ptr<actions::Task> > _actions_set;

TaskMap _map;
memory::Manager _memory;

mwoibn::Matrix _P;
double _dofs, _dt;

};
}
} // namespace package
} // namespace library

#endif
