#include "mwoibn/hierarchical_control/hierarchical_controller_continous.h"
#include <cmath>
#include <boost/range/adaptor/reversed.hpp>

void mwoibn::hierarchical_control::HierarchicalControllerContinous::addTask(
    ControllerTask* new_task, mwoibn::VectorN gain, int i, double damp)
{

  mwoibn::hierarchical_control::HierarchicalController::addTask(new_task, gain,
                                                                i,damp);
  _resize();
  _last_stack_change = 0;
}

void mwoibn::hierarchical_control::HierarchicalControllerContinous::_resize()
{
  int size = 0;

  for (auto& task : _tasks_ptr)
  {
    size += task->getTaskSize();
  }

  _e.setZero(size);
  _g.setZero(_robot.getDofs(), size);
}

void mwoibn::hierarchical_control::HierarchicalControllerContinous::removeTask(
    int i)
{
  mwoibn::hierarchical_control::HierarchicalController::removeTask(i);

  _resize();
  _last_stack_change = 0;
}

const mwoibn::VectorN &mwoibn::hierarchical_control::HierarchicalControllerContinous::update()
{
  for (auto& task : _tasks_ptr)
  {
    task->update();
  }
  _previous_joint_states.noalias() = _joint_states;

  compute();

  return getCommand();
}

void mwoibn::hierarchical_control::HierarchicalControllerContinous::compute()
{
  _command.setZero();
  _P.setIdentity();
  int i = 0;
  for (auto& task : _tasks_ptr)
  {
    _updateTask(i, task);
    ++i;
  }

  if (!_checkStack()){
      _last_stack_change = 0;
    std::cout << "stack change detected" << std::endl;
  }

  if (!_last_stack_change)
    _resetCorrection();

  _g.setZero();
  int cols = _g.cols();
  _g_it.setIdentity();
  i--;
  for (auto& task : boost::adaptors::reverse(_tasks_ptr))
  {
    cols -= task->getTaskSize();

    __n_by_n1.setIdentity();
//    std::cout << "inverser\n" << _inversers_ptrs[i]->getInverse().rows() << "x" << _inversers_ptrs[i]->getInverse().cols() << std::endl;
//    std::cout << "jacobian\n" << task->getJacobian().rows() << "x" << task->getJacobian().cols() << std::endl;
    __n_by_n2.noalias() = _inversers_ptrs[i]->getInverse() * task->getJacobian();
    __n_by_n1.noalias() -= __n_by_n2;
    __n_by_n2.noalias() = _g_it * __n_by_n1;

    _g.block(0, cols, _robot.getDofs(), task->getTaskSize()) =
        _g_it * _inversers_ptrs[i]->getInverse();
    i--;
  }

  ++_last_stack_change;

  // change with respect to the original
  _command.noalias() +=
      _g * _e * exp(-_mu * _last_stack_change *
          _robot.rate()); //! \todo ** frequency should be an outside argument
}

void mwoibn::hierarchical_control::HierarchicalControllerContinous::
    _resetCorrection()
{
//  std::cout << "_resetCorrection" << std::endl;

  int i = 0;
  int size = 0;
  for (auto& task : _tasks_ptr)
  {
//    std::cout << "error " <<  _gains[i].cwiseProduct(task->getPreviousError()) << std::endl;
//    std::cout << "d error " << task->getPreviousJacobian() * _previous_joint_states << std::endl;

//    std::cout << _e.size() << std::endl;
//    std::cout << _e.segment(size, task->getTaskSize()) << std::endl;
    _e.segment(size, task->getTaskSize()) =
        _gains[i].cwiseProduct(task->getError());
    _e.segment(size, task->getTaskSize()) +=
        task->getJacobian() * _joint_states;
    size += task->getTaskSize();
    i++;
  }
}

