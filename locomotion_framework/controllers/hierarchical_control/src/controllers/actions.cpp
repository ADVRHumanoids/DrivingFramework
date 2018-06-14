#include "mwoibn/hierarchical_control/controllers/actions.h"
#include "mwoibn/hierarchical_control/actions/compute.h"


mwoibn::hierarchical_control::controllers::Actions::Actions(double dt, unsigned int dofs) : Basic(), _dt(dt), _dofs(dofs) {
        _command = mwoibn::VectorN::Zero(_dofs);
        _P = mwoibn::Matrix::Identity(_dofs, _dofs);
        //
        // preallocate three snap actions
        int snap = 4, replace = 1, secondary = 8;
        _snap.reserve(snap);

        for(int i = 0; i < snap; i++)
                _snap.push_back(actions::Snap(_P, _command, _memory));
        _memory.release(_snap);

        _replace.reserve(replace);

        for(int i = 0; i < replace; i++)
                _replace.push_back(actions::Replace(_P, _command, 5, _dt, _memory, _map));
        _memory.release(_replace);

        _secondary.reserve(secondary);

        for(int i = 0; i < secondary; i++)
                _secondary.push_back(actions::Secondary(_memory, _map));

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

        _map[&new_task] = _actions_set.back().get();
}

void mwoibn::hierarchical_control::controllers::Actions::_addTask(tasks::BasicTask& new_task, double gain,
                                                                  double damping){
        _actions_set.push_back(std::unique_ptr<actions::Task>(new actions::Compute(new_task, gain, damping, _P, _command, _memory)));
        _map[&new_task] = _actions_set.back().get();
}

bool mwoibn::hierarchical_control::controllers::Actions::_setIdle(tasks::BasicTask& task){
        if(!_memory.isSecondary()) return false;
        actions::Secondary* ptr = _memory.getSecondary();
        ptr->assign(*_map[&task]);
        _map[&task] = ptr;
}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, mwoibn::VectorN gain, double damping){
        _addTask(new_task, gain, damping);
        _active_stack.push_back(_map[&new_task]);

        LOG_INFO << "add task at the end of the stack" << std::endl;
}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, double gain, double damping){
        _addTask(new_task, gain, damping);
        _active_stack.push_back(_map[&new_task]);

        LOG_INFO << "add task at the end of the stack" << std::endl;
}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, unsigned int i, mwoibn::VectorN gain, double damping){

        if( i > _map.size())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, received a parameter beyond the stack size"));

        _addTask(new_task, gain, damping);
        _active_stack.insert(_active_stack.begin()+i, _map[&new_task]);

        LOG_INFO << "add task acording to the reference" << std::endl;

}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, unsigned int i, double gain, double damping){

        if( i > _map.size())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, received a parameter beyond the stack size"));

        _addTask(new_task, gain, damping);
        _active_stack.insert(_active_stack.begin()+i, _map[&new_task]);

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

bool mwoibn::hierarchical_control::controllers::Actions::replace(tasks::BasicTask& task, double mu){

        std::cout << "replace" << std::endl;
        if(!_memory.isReplace()) return false;
        std::cout << "it's free!" << std::endl;
        if(!_memory.isSnap()) return false;
        // std::cout << "snap's there!" << std::endl;
        if(!_memory.isSecondary()) return false;
        // std::cout << "free secondary!" << std::endl;

        actions::Replace* ptr = _memory.getReplace();
        tasks::BasicTask& task_old = _active_stack.back()->baseAction().getTask();

        ptr->start(_map[&task]->next(), _map[&task_old]->next(), *_memory.getSnap(), mu);
        _map[&task_old]->swap(*ptr);
        _map[&task] = ptr;

        _setIdle(_active_stack.back()->baseAction().getTask());

        return true;
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
        for (auto& pair : _map)
        {
                pair.first->update();
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
        return _map[&task]->baseAction().updateGain(gain);
}
bool mwoibn::hierarchical_control::controllers::Actions::updateGain(tasks::BasicTask& task, double gain){
        return _map[&task]->baseAction().updateGain(gain);
}
