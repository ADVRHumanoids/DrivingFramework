#ifndef __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS__ACTIONS_H
#define __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS__ACTIONS_H


#include "mwoibn/hierarchical_control/controllers/basic.h"

#include "mwoibn/hierarchical_control/actions/basic.h"
#include "mwoibn/hierarchical_control/actions/task.h"
#include "mwoibn/hierarchical_control/maps/actions_map.h"

#include "mwoibn/hierarchical_control/actions/idle.h"
#include "mwoibn/hierarchical_control/actions/snap.h"
#include "mwoibn/hierarchical_control/actions/replace.h"
#include "mwoibn/hierarchical_control/actions/static_replace.h"

#include "mwoibn/hierarchical_control/memory/manager.h"
#include "mwoibn/hierarchical_control/state.h"


namespace mwoibn {
namespace hierarchical_control {
namespace controllers {

class Actions : public Basic
{
public:
Actions(double dt, unsigned int dofs); // what should be allocated should come from external sources, also type of the action
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

// add action ad the end of the stack
virtual void addAction(actions::Task& task){
        if(_map.exist(task.baseAction().getTask()))
                _map[task.baseAction().getTask()]->swap(task);
        else
                _map.swap(task.baseAction().getTask(), task);
        _active_stack.push_back(&task);

        std::cout << "hierarchical_control::addAction: added action at the end of the stack" << std::endl;
}

virtual void addAfter(actions::Task& new_task, actions::Task& old_task){
        if(!_map.exist(old_task.baseAction().getTask()))
          throw std::runtime_error(std::string("Could not add action to the stack , unknown reference task."));


          // it should check if tasks has already been in the stack
        _map[new_task.baseAction().getTask()] = &new_task;

        auto i = ranges::find(_active_stack, &old_task);

        if(i == ranges::end(_active_stack))
          throw std::runtime_error(std::string("Could not add action to the stack , unknown reference task."));

        i++;
        _active_stack.insert( i, &new_task );

        std::cout << "hierarchical_control::addAction: added action at given position " << std::endl;
}


maps::ActionsMap& map(){
        return _map;
}

actions::Task& getAction(tasks::BasicTask& task){
        return *_map[task];
}

State state;

protected:
// virtual void _construct() = 0;
void _addTask(tasks::BasicTask& new_task, double gain, double damping);
void _addTask(tasks::BasicTask& new_task, mwoibn::VectorN& gain, double damping);
bool _setIdle(tasks::BasicTask& new_task);

std::vector<actions::Task* > _active_stack;

std::vector<actions::Snap > _snap;
std::vector<actions::StaticReplace > _replace;
std::vector<actions::Secondary > _secondary;

std::vector<std::unique_ptr<actions::Task> > _actions_set;

maps::ActionsMap _map;
memory::Manager _memory;

mwoibn::Matrix _P;
double _dofs, _dt;




};
}
} // namespace package
} // namespace library

#endif
