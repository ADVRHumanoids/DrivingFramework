#include "mwoibn/hierarchical_control/controllers/actions.h"
#include "mwoibn/hierarchical_control/actions/compute.h"


void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, mwoibn::VectorN gain, double damping){
        if(gain.size() != new_task.getTaskSize())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, wrong gains vector size."));

        if (_map.size() == 0) _init(new_task);

        _map[&new_task] = actions_set();
        _map[&new_task].push_back(std::unique_ptr<actions::Basic> (new actions::Compute(new_task, gain, damping, _P, _command)));

        _active_stack.push_back(*_map[&new_task][0]);

        LOG_INFO << "add task at the end of the stack" << std::endl;
}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, double gain, double damping){
        mwoibn::VectorN set_gain;
        set_gain.setConstant(new_task.getTaskSize(), gain);

        if (_map.size() == 0) _init(new_task);

        _map[&new_task] = actions_set();
        _map[&new_task].push_back(std::unique_ptr<actions::Basic> (new actions::Compute(new_task, gain, damping, _P, _command)));

        _active_stack.push_back(*_map[&new_task][0]);

        LOG_INFO << "add task at the end of the stack" << std::endl;

}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, unsigned int i, mwoibn::VectorN gain, double damping){
        if(gain.size() != new_task.getTaskSize())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, wrong gains vector size."));

        if (_map.size() == 0) _init(new_task);
        if( i > _map.size())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, received a parameter beyond the stack size"));

        _map[&new_task] = actions_set();
        _map[&new_task].push_back(std::unique_ptr<actions::Basic> (new actions::Compute(new_task, gain, damping, _P, _command)));


        _active_stack.insert(_active_stack.begin()+i, *_map[&new_task][0]);

        LOG_INFO << "add task acording to the reference" << std::endl;

}

void mwoibn::hierarchical_control::controllers::Actions::addTask(tasks::BasicTask& new_task, unsigned int i, double gain, double damping){
        mwoibn::VectorN set_gain;
        set_gain.setConstant(new_task.getTaskSize(), gain);

        if (_map.size() == 0) _init(new_task);
        if( i > _map.size())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, received a parameter beyond the stack size"));

        _map[&new_task] = actions_set();
        _map[&new_task].push_back(std::unique_ptr<actions::Basic> (new actions::Compute(new_task, gain, damping, _P, _command)));

        _active_stack.insert(_active_stack.begin()+i, *_map[&new_task][0]);

        LOG_INFO << "add task acording to the reference" << std::endl;

}

void mwoibn::hierarchical_control::controllers::Actions::_init(tasks::BasicTask& task){
        _dofs = task.getTaskDofs();
        _command = mwoibn::VectorN::Zero(_dofs);
        _P = mwoibn::Matrix::Identity(_dofs, _dofs);
}

void mwoibn::hierarchical_control::controllers::Actions::removeTask(unsigned int i)
{
        if (i < _active_stack.size())
        {
                removeTask(_active_stack[i].get().getTask());
        }
        else
                LOG_INFO << "Got wrong data, controller " << i
                         << "is not defined in the controller. Only " << _active_stack.size()
                         << " tasks is deifined in the controller." << std::endl;
}

void mwoibn::hierarchical_control::controllers::Actions::removeTask(tasks::BasicTask& task){
        for(auto& action_ptr : _map[&task]) {
                _active_stack.erase(std::remove_if(_active_stack.begin(), _active_stack.end(), [&action_ptr](auto action){return action_ptr.get() == &action.get(); }), _active_stack.end());
        }
        _map.erase(_map.find (&task));
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
        {
                action.get().run();
        }

}

bool mwoibn::hierarchical_control::controllers::Actions::updateGain(tasks::BasicTask& task, const mwoibn::VectorN& gain){
        return _map[&task][0]->baseAction().updateGain(gain);

}
bool mwoibn::hierarchical_control::controllers::Actions::updateGain(tasks::BasicTask& task, double gain){
        return _map[&task][0]->baseAction().updateGain(gain);
}
