#include "mwoibn/hierarchical_control/controllers/default.h"

void mwoibn::hierarchical_control::controllers::Default::addTask(tasks::BasicTask& new_task, mwoibn::VectorN gain, double damping){
        if(gain.size() != new_task.getTaskSize())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, wrong gains vector size."));
        mwoibn::VectorN error = mwoibn::VectorN::Zero(new_task.getTaskSize());
        mwoibn::Matrix jacobian =
                mwoibn::Matrix::Zero(new_task.getTaskSize(), new_task.getTaskDofs());

        LOG_INFO << "add task at the end of the stack" << std::endl;
        _tasks.push_back(new_task);
        _gains.push_back(gain);
        _errors.push_back(error);
        _inversers_ptrs.push_back(std::unique_ptr<mwoibn::Projection>(
                                          new mwoibn::Projection(jacobian, damping)));

        if (size() == 1) _init();


}
void mwoibn::hierarchical_control::controllers::Default::addTask(tasks::BasicTask& new_task, double gain, double damping){
        mwoibn::VectorN set_gain;
        set_gain.setConstant(new_task.getTaskSize(), gain);
        mwoibn::VectorN error = mwoibn::VectorN::Zero(new_task.getTaskSize());
        mwoibn::Matrix jacobian =
                mwoibn::Matrix::Zero(new_task.getTaskSize(), new_task.getTaskDofs());

        LOG_INFO << "add task at the end of the stack" << std::endl;
        _tasks.push_back(new_task);
        _gains.push_back(set_gain);
        _errors.push_back(error);
        _inversers_ptrs.push_back(std::unique_ptr<mwoibn::Projection>(
                                          new mwoibn::Projection(jacobian, damping)));

        if (size() == 1) _init();

}
void mwoibn::hierarchical_control::controllers::Default::addTask(tasks::BasicTask& new_task, unsigned int i, mwoibn::VectorN gain, double damping){
        if(gain.size() != new_task.getTaskSize())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, wrong gains vector size."));

        mwoibn::VectorN error = mwoibn::VectorN::Zero(new_task.getTaskSize());
        mwoibn::Matrix jacobian =
                mwoibn::Matrix::Zero(new_task.getTaskSize(), new_task.getTaskDofs());

        if(i > size())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, given position is greater than the stack size."));

        LOG_INFO << "add task acording to the refrence" << std::endl;
        _tasks.insert(_tasks.begin() + i, new_task);
        _gains.insert(_gains.begin() + i, gain);
        _errors.insert(_errors.begin() + i, error);
        _inversers_ptrs.insert(_inversers_ptrs.begin() + i, std::unique_ptr<mwoibn::Projection>(
                                       new mwoibn::Projection(jacobian, damping)));

        if (size() == 1) _init();
}
void mwoibn::hierarchical_control::controllers::Default::addTask(tasks::BasicTask& new_task, unsigned int i, double gain, double damping){
        mwoibn::VectorN set_gain;
        set_gain.setConstant(new_task.getTaskSize(), gain);

        mwoibn::VectorN error = mwoibn::VectorN::Zero(new_task.getTaskSize());
        mwoibn::Matrix jacobian =
                mwoibn::Matrix::Zero(new_task.getTaskSize(), new_task.getTaskDofs());

        if(i > size())
                throw(std::invalid_argument("hierarchical_control::controllers::Default - couldn't add the task, given position is greater than the stack size."));

        LOG_INFO << "add task acording to the refrence" << std::endl;
        _tasks.insert(_tasks.begin() + i, new_task);
        _gains.insert(_gains.begin() + i, set_gain);
        _errors.insert(_errors.begin() + i, error);
        _inversers_ptrs.insert(_inversers_ptrs.begin() + i, std::unique_ptr<mwoibn::Projection>(
                                       new mwoibn::Projection(jacobian, damping)));

        if (_tasks.size() == 1) _init();
}

void mwoibn::hierarchical_control::controllers::Default::_init(){
        _dofs = _tasks.front().get().getTaskDofs();
        _command = mwoibn::VectorN::Zero(_dofs);
        _P = mwoibn::Matrix::Identity(_dofs, _dofs);
}

void mwoibn::hierarchical_control::controllers::Default::removeTask(unsigned int i)
{
        if (i < size())
        {
                _tasks.erase(_tasks.begin() + i);
                _gains.erase(_gains.begin() + i);
                _errors.erase(_errors.begin() + i);
                _inversers_ptrs.erase(_inversers_ptrs.begin() + i);

                if (i == 0 & size() > 0)
                        _dofs = _tasks.front().get().getTaskDofs();
        }
        else
                LOG_INFO << "Got wrong data, controller " << i
                         << "is not defined in the controller. Only " << size()
                         << " tasks is deifined in the controller." << std::endl;
}

const mwoibn::VectorN&
mwoibn::hierarchical_control::controllers::Default::update()
{
        for (auto& task : _tasks)
        {
                task.get().update();
        }
		
        compute();
		
        return getCommand();
}

void mwoibn::hierarchical_control::controllers::Default::compute()
{
        _command.setZero();
        _P.setIdentity();

        int i = 0;

        for (auto& task : _tasks)
        {
                _updateTask(i, task);
                ++i;
        }

}

void mwoibn::hierarchical_control::controllers::Default::_updateTask(int i, mwoibn::hierarchical_control::tasks::BasicTask& task){
        _errors[i].noalias() = -(_gains[i].asDiagonal() * task.getError());
        _errors[i].noalias() -= task.getJacobian() * _command;

        if (_errors[i].size())
        {
                _inversers_ptrs[i]->compute(task.getJacobian(), _P);
                _command.noalias() += _inversers_ptrs[i]->getInverse() * _errors[i];
        }
}
