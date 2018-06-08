#include "mwoibn/hierarchical_control/hierarchical_controller.h"

void mwoibn::hierarchical_control::HierarchicalController::addTask(
    mwoibn::hierarchical_control::ControllerTask* new_task,
    mwoibn::VectorN gain, int i, double damping)
{

  mwoibn::VectorN set_gain;
  set_gain.resize(new_task->getError().size());

  if (gain.size() == 1)
  {
    set_gain = gain[0] * mwoibn::VectorN::Ones(new_task->getError().size());
  }
  else if (gain.size() == new_task->getError().size())
  {
    set_gain = gain;
  }
  else
    LOG_INFO
        << "Wrong gain size, it should be a scalar or of a size of the task"
        << std::endl;

  mwoibn::VectorN error = mwoibn::VectorN::Zero(new_task->getTaskSize());
  mwoibn::Matrix jacobian =
      mwoibn::Matrix::Zero(new_task->getTaskSize(), new_task->getTaskDofs());

  if (!_tasks_ptr.size())
    i = 0;
  if ((i < 0) || (i >= _tasks_ptr.size()))
  {
    LOG_INFO << "add task at the end of the stack" << std::endl;
    _tasks_ptr.push_back(new_task);
    _gains.push_back(set_gain);
    _errors.push_back(error);
    _inversers_ptrs.push_back(std::unique_ptr<mwoibn::Projection>(
        new mwoibn::Projection(jacobian, damping)));
  }
  else
  {
    LOG_INFO << "add task acording to the refrence" << std::endl;
    _tasks_ptr.insert(_tasks_ptr.begin() + i, new_task);
    _gains.insert(_gains.begin() + i, set_gain);
    _errors.insert(_errors.begin() + i, error);
    _inversers_ptrs.insert(_inversers_ptrs.begin() + i, std::unique_ptr<mwoibn::Projection>(
        new mwoibn::Projection(jacobian, damping)));
  }
  if (i == 0)
  {
    _dofs = new_task->getTaskDofs();
    _command = mwoibn::VectorN::Zero(_dofs);
    _P = mwoibn::Matrix::Identity(_dofs, _dofs);
  }
}
void mwoibn::hierarchical_control::HierarchicalController::removeTask(int i)
{
  if ((i >= 0) && (i < _tasks_ptr.size()))
  {
    _tasks_ptr.erase(_tasks_ptr.begin() + i);
    _gains.erase(_gains.begin() + i);
    _errors.erase(_errors.begin() + i);
    _inversers_ptrs.erase(_inversers_ptrs.begin() + i);

    if (i == 0 & _tasks_ptr.size() > 0)
      _dofs = _tasks_ptr.front()->getTaskDofs();
  }
  else
    LOG_INFO << "Got wrong data, controller " << i
             << "is not defined in the controller. Only " << _tasks_ptr.size()
             << " tasks is deifined in the controller." << std::endl;
}

const mwoibn::VectorN&
mwoibn::hierarchical_control::HierarchicalController::update()
{
  for (auto& task : _tasks_ptr)
  {
    task->update();
  }
  compute();

  return getCommand();
}

void mwoibn::hierarchical_control::HierarchicalController::compute()
{
  _command.setZero();
  _P.setIdentity();

  int i = 0;

  for (auto& task : _tasks_ptr)
  {
    _updateTask(i, task);
    ++i;

  }

}

void mwoibn::hierarchical_control::HierarchicalController::_updateTask(int i, mwoibn::hierarchical_control::ControllerTask* task){
  _errors[i].noalias() = -(_gains[i].asDiagonal() * task->getError());
  _errors[i].noalias() -= task->getJacobian() * _command;

  if (_errors[i].size())
  {
          _inversers_ptrs[i]->compute(task->getJacobian(), _P);
          _command.noalias() += _inversers_ptrs[i]->getInverse() * _errors[i];
  }
}
