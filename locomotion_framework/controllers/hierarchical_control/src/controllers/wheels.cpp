#include "mwoibn/hierarchical_control/controllers/wheels.h"
#include "mwoibn/hierarchical_control/tasks/castor_angle_task.h"
#include <boost/range/adaptor/reversed.hpp>

bool mwoibn::hierarchical_control::controllers::Wheels::_checkStack()
{
        _last_state.noalias() = _state;
        bool stack = true;
        for (int i = 0; i < _size; i++)
        {
//    bool high =
//        _inversers_ptrs[_id_high]->getJacobian().row(i).isApproxToConstant(
//            0, 1e-3);
//    bool low =
//        _inversers_ptrs[_id_low]->getJacobian().row(i).isApproxToConstant(0,
//                                                                          1e-3);

                if (_state[i] && _inversers_ptrs[_id_high]->getJacobian().row(i).cwiseAbs().maxCoeff() < 6e-3)
                {
                        //      std::cout << "!high\t" <<
                        //      _inversers_ptrs[_id_high]->getJacobian().row(i) << std::endl;
                        _state[i] = 0;
//      _values[i] = 0;
                }
                else if (!_state[i] && _inversers_ptrs[_id_high]->getJacobian().row(i).cwiseAbs().maxCoeff() > 8e-3)
                {
                        //      std::cout << "!high\t" <<
                        //      _inversers_ptrs[_id_high]->getJacobian().row(i) << std::endl;
                        _state[i] = 1;
//      _values[i] = 0;
                }
//    else if ()_state[i] = 1;
//    else if (low)
//    {
//      //      std::cout << "!low\t" <<
//      //      _inversers_ptrs[_id_low]->getJacobian().row(i) << std::endl;

//      _state[i] = 1;
//      _values[i] = 100;
//    }
//    else
//    {
//      //      std::cout << "compare" << std::endl;
//      double max_high =
//          _inversers_ptrs[_id_high]->getJacobian().row(i).cwiseAbs().maxCoeff();
//      double max_low =
//          _inversers_ptrs[_id_low]->getJacobian().row(i).cwiseAbs().maxCoeff();

                //      std::cout << "max_high" << max_high << std::endl;
                //      std::cout << "max_low" << max_low<< std::endl;

//      double value = max_high / max_low;
//      if (value > 1.05)
//        _state[i] = 1;
//      else if (value < 0.95)
//        _state[i] = 0;

//      _values[i] = value;

//          std::cout << "high\t" << high << "\t" <<
//      //              <<
//      //              << "\t" <<
//                    _inversers_ptrs[_id_high]->getJacobian().row(i)
//                    << std::endl;
//          std::cout << "jacobian\t" << high << "\t" <<
//      //              <<
//      //              << "\t" <<
//                    _task_high.getJacobian().row(i)
//                    << std::endl;

//          std::cout << "low\t" << low << "\t"
//                    <<
//                    _inversers_ptrs[_id_low]->getJacobian().row(i)  << std::endl;
//    }

                if(_state[i] != _last_state[i]) {
                        tasks::CastorAngleTask& cast = dynamic_cast<tasks::CastorAngleTask&>(_task_low);
                        if(!_state[i])
                                cast.setReference(i, cast.getCurrent()[i]);
//      else
//        cast.setReference(i, -mwoibn::PI);
                }

                stack = stack && (_state[i] == _last_state[i]);
        }

////  if (!stack)
////  {
//    std::cout << _values.transpose() << ",\t" << _state.transpose()
//              << std::endl;
        std::cout << _state.transpose() << "\t";
//                << std::endl;
        for (int i = 0; i < _size; i++) {
                std::cout
                << "high: "
                << _inversers_ptrs[_id_high]->getJacobian().row(i).cwiseAbs().maxCoeff()
                << ",\t"
                << " low: "
                << _inversers_ptrs[_id_low]->getJacobian().row(i).cwiseAbs().maxCoeff()         << ",\t";
        }
//    }
//    std::cout << std::endl;
//  //  std::cout << stack << std::endl;

        return stack;
}

void mwoibn::hierarchical_control::controllers::Wheels::_findTasks()
{
        auto it = std::find_if(_tasks.begin(), _tasks.end(), [this](auto task){ return &task.get() == &(this->_task_high); });

        if (it == _tasks.end())
                throw(std::invalid_argument("mwoibn::hierarchical_control::controllers::Wheels: couldn't find a "
                                            "task_high in a tasks stack."));
        _id_high = it - _tasks.begin();

        it = std::find_if(_tasks.begin(), _tasks.end(), [this](auto task){ return &task.get() == &(this->_task_low); });
        // it = std::find(_tasks.begin(), _tasks.end(), _task_low);

        if (it == _tasks.end())
                throw(std::invalid_argument("mwoibn::hierarchical_control::controllers::Wheels: couldn't find a "
                                            "task_low in a tasks stack."));
        _id_low = it - _tasks.begin();
}

//void mwoibn::hierarchical_control::HierarchicalControllerWheels::_updateTask(
//    int i, mwoibn::hierarchical_control::BasicTask* task)
//{
//  _errors[i].noalias() = -(_gains[i].asDiagonal() * task->getError());
//  _errors[i].noalias() -= task->getJacobian() * _command;

//  if (_errors[i].size())
//  {
//    _inversers_ptrs[i]->compute(task->getJacobian(), _P);
//    _command.noalias() += _inversers_ptrs[i]->getInverse() * _errors[i];
//  }
//}


void mwoibn::hierarchical_control::controllers::Wheels::compute()
{
        _command.setZero();
        _P.setIdentity();

        mwoibn::Matrix P_1 = _P, P_2 = _P;
        int i = 0;

        for (auto& task : _tasks)
        {
                if (_errors[i].size())
                {
                        if(&task.get() == &_task_high) {
                                std::cout << i << std::endl;
                                P_1 = _P; // just for now
                        }
                        if(&task.get() == &_task_low)
                                P_2 = _P;  // just for now
                        _inversers_ptrs[i]->compute(task.get().getJacobian(), _P);
                }
                ++i;
        }

        if (!_checkStack()) {
                _P = P_2;
                _task_low.update();
                _inversers_ptrs[_id_low]->compute(_task_low.getJacobian(), _P);

                _last_stack_change = 0;
                std::cout << "stack change detected" << std::endl;
        }

//  i = 0;

        for (int k = 0; k < _tasks.size()-2; k++)
        {
                auto& task = _tasks[k];

                _errors[k].noalias() = -(_gains[k].asDiagonal() * task.get().getError());
                _errors[k].noalias() -= task.get().getJacobian() * _command;

                _command.noalias() += _inversers_ptrs[k]->getInverse() * _errors[k];

        }

//  i = _tasks_ptr.size()-2;
        _errors[_id_high].noalias() = -(_gains[_id_high].asDiagonal() * _task_high.getError());
        _errors[_id_high].noalias() -= _task_high.getJacobian() * _command;
        _command.noalias() += _inversers_ptrs[_id_high]->getInverse() * _errors[_id_high];



        _errors[_id_low].noalias() = -(_gains[_id_low].asDiagonal() * _task_low.getError());
        _errors[_id_low].noalias() -= _task_low.getJacobian() * _command;

        for (int j = 0; j < _task_low.getTaskSize(); j++) {
                if(_state[j])
                        _errors[_id_low][j] = 0;
        }

        _command.noalias() += _inversers_ptrs[_id_low]->getInverse() * _errors[_id_low];


        if (!_last_stack_change)
                _resetCorrection();

        _g.setZero();
        int cols = _g.cols();
        _g_it.setIdentity();
        i = _tasks.size()-1;
        for (auto& task : boost::adaptors::reverse(_tasks))
        {
                cols -= task.get().getTaskSize();

                __n_by_n1.setIdentity();
                __n_by_n2.noalias() = _inversers_ptrs[i]->getInverse() * task.get().getJacobian();
                __n_by_n1.noalias() -= __n_by_n2;
                __n_by_n2.noalias() = _g_it * __n_by_n1;

                _g_it.noalias() = __n_by_n2;

                _g.block(0, cols, _robot.getDofs(), task.get().getTaskSize()) =
                        _g_it * _inversers_ptrs[i]->getInverse();
                i--;
        }

        ++_last_stack_change;

        std::cout << "exp" << exp(-_mu * _last_stack_change *_robot.rate()) << std::endl;
//   change with respect to the original
        _command.noalias() +=
                _g * _e * exp(-_mu * _last_stack_change *
                              _robot.rate()); //! \todo ** frequency should be an outside argument
}
