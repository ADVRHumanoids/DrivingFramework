# include "mwoibn/hierarchical_control/controllers/actions.h"
#include "mwoibn/hierarchical_control/actions/compute.h"


mwoibn::hierarchical_control::controllers::Actions::Actions(double dt, unsigned int dofs) : Basic(), _dt(dt), _dofs(dofs), _state(_command, _P, _map, _memory, _dt, _dofs) {
        _command = mwoibn::VectorN::Zero(_dofs);
        _P = mwoibn::Matrix::Identity(_dofs, _dofs);

        int snap = 4, replace = 1, secondary = 8;

        _snap.assign(snap, actions::Snap(_P, _command, _memory));
        _replace.assign(replace, actions::StaticReplace(_P, _command, 5, _dt, _memory, _map));
        _secondary.assign(secondary,actions::Secondary(_memory, _map));

        _memory.release(_snap);
        _memory.release(_replace);
        _memory.release(_secondary);

}

void mwoibn::hierarchical_control::controllers::Actions::idleTask(tasks::BasicTask& new_task, mwoibn::VectorN gain,
                                                                  double damping){
        _addTask(new_task, gain, damping);
        _setIdle(new_task);
}

void mwoibn::hierarchical_control::controllers::Actions::idleTask(tasks::BasicTask& new_task, double gain,
                                                                  double damping){
        _addTask(new_task, gain, damping);
        _setIdle(new_task);
}

void mwoibn::hierarchical_control::controllers::Actions::_addTask(tasks::BasicTask& new_task, mwoibn::VectorN& gain,
                                                                  double damping){
        _actions_set.push_back(std::unique_ptr<actions::Task>(new actions::Compute(new_task, gain, damping, _P, _command, _memory)));

        _map[new_task] = _actions_set.back().get();
}

void mwoibn::hierarchical_control::controllers::Actions::_addTask(tasks::BasicTask& new_task, double gain,
                                                                  double damping){
        _actions_set.push_back(std::unique_ptr<actions::Task>(new actions::Compute(new_task, gain, damping, _P, _command, _memory)));
        _map[new_task] = _actions_set.back().get();
}

bool mwoibn::hierarchical_control::controllers::Actions::_setIdle(tasks::BasicTask& task){
        if(!_memory.secondary.is()) return false;
        actions::Secondary* ptr = _memory.secondary.get();
        ptr->assign(*_map[task]);
        _map[task] = ptr;
}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, mwoibn::VectorN gain, double damping){
        _addTask(new_task, gain, damping);
        _active_stack.push_back(_map[new_task]);

        LOG_INFO << "add task at the end of the stack" << std::endl;
}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, double gain, double damping){
        _addTask(new_task, gain, damping);
        _active_stack.push_back(_map[new_task]);

        LOG_INFO << "add task at the end of the stack" << std::endl;
}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, unsigned int i, mwoibn::VectorN gain, double damping){

        if( i > _map.size())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, received a parameter beyond the stack size"));

        _addTask(new_task, gain, damping);
        _active_stack.insert(_active_stack.begin()+i, _map[new_task]);

        LOG_INFO << "add task acording to the reference" << std::endl;

}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, unsigned int i, double gain, double damping){

        if( i > _map.size())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, received a parameter beyond the stack size"));

        _addTask(new_task, gain, damping);
        _active_stack.insert(_active_stack.begin()+i, _map[new_task]);

        LOG_INFO << "add task acording to the reference" << std::endl;

}

void mwoibn::hierarchical_control::controllers::Actions::removeTask(unsigned int i)
{
        // // check if primary task or not
        // if (i < _active_stack.size())
        // {
        //         removeTask(_active_stack[i].get().baseAction().getTask());
        // }
        // else
        //         LOG_INFO << "Got wrong data, controller " << i
        //                  << "is not defined in the controller. Only " << _active_stack.size()
        //                  << " tasks is deifined in the controller." << std::endl;
}
//
// actions::Snap* mwoibn::hierarchical_control::controllers::Actions::_assignSnap(actions::Basic& action){
//
//         if(!_idle_snap.size()) return;
//
//         actions::Snap* snap = _idle_snap[0];
//         _idle_snap.erase(_idle_snap.begin());
//
//         return snap;
// }

mwoibn::hierarchical_control::actions::Replace* mwoibn::hierarchical_control::controllers::Actions::replace(tasks::BasicTask& task, double mu){

        if(!_memory.replace.is()) return nullptr;
        if(!_memory.snap.is()) return nullptr;
        if(!_memory.secondary.is()) return nullptr;

        actions::Replace* ptr = _memory.replace.get();
        tasks::BasicTask& task_old = _active_stack.back()->baseAction().getTask();

        ptr->start(_map[task]->next(), _map[task_old]->next(), *_memory.snap.get(), mu);
        _map[task_old]->swap(*ptr);
        _map[task] = ptr;

        _setIdle(_active_stack.back()->baseAction().getTask());

        return ptr;
}

void mwoibn::hierarchical_control::controllers::Actions::removeTask(tasks::BasicTask& task){
        // for(auto& action_ptr : _map[&task]) {
        //         _active_stack.erase(std::remove_if(_active_stack.begin(), _active_stack.end(), [&action_ptr](auto action){return action_ptr.get() == &action.get(); }), _active_stack.end());
        // }
        // _map.erase(_map.find (&task));
}


const mwoibn::VectorN&
mwoibn::hierarchical_control::controllers::Actions::update()
{
        for (maps::ActionsMap::key_iter it = _map.keyBegin(); it!=_map.keyEnd(); it++)
        {
                (*it)->update();
        }
        compute();

        return getCommand();
}

void mwoibn::hierarchical_control::controllers::Actions::compute()
{
        _command.setZero();
        _P.setIdentity();

        for (auto& action : _active_stack)
                action->run();

        for (auto& action : _active_stack) {
                action = &action->next();
                //if(action == next) continue;
                //action->release();
                //action = next;
        }
}

bool mwoibn::hierarchical_control::controllers::Actions::updateGain(tasks::BasicTask& task, const mwoibn::VectorN& gain){
        return _map[task]->baseAction().updateGain(gain);
}
bool mwoibn::hierarchical_control::controllers::Actions::updateGain(tasks::BasicTask& task, double gain){
        return _map[task]->baseAction().updateGain(gain);
}
