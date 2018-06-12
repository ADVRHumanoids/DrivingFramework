#ifndef __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_ACTIONS_H
#define __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_ACTIONS_H


#include "mwoibn/hierarchical_control/controllers/basic.h"
#include "mwoibn/hierarchical_control/actions/basic_action.h"

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
 * \todo add basic cartesian position task
 * \todo add a constructor taking the tasks (with variable number of arguments)
 * \todo **extract the continous adjustement to the task change as an inheriting
 *******************************class**
 *
 */
class Actions : public Basic
{
public:
Actions() : Basic() {
}
virtual ~Actions() {
}
virtual void init(){
        _construct();
}

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
virtual void removeTask(tasks::BasicTask& task);

/**
 * \see Basic#compute()
 *
 */
virtual const mwoibn::VectorN& update() = 0;
//! computes the controll law without updating the controllers states
/**
 * \see Basic#updateController()
 *
 */
virtual void compute() = 0;

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
virtual void _construct() = 0;
void _init(tasks::BasicTask& task);
std::vector<std::reference_wrapper<actions::Basic> > _active_stack;
// std::vector<std::unique_ptr<actions::Basic> > _actions_ptr;
typedef std::vector<std::unique_ptr<actions::Basic> > actions_set;
std::map<tasks::BasicTask*, actions_set> _map;


mwoibn::Matrix _P;
double _dofs;
//typedef boost::bimap<std::reference_wrapper<tasks::BasicTask>, std::reference_wrapper<actions::Basic> > boost_map;
// boost_map _map;
};
}
} // namespace package
} // namespace library

#endif
