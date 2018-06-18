#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_REPLACE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_REPLACE_H

#include "mwoibn/hierarchical_control/actions/basic_action.h"
#include "mwoibn/hierarchical_control/actions/secondary.h"
#include "mwoibn/hierarchical_control/actions/task.h"
#include "mwoibn/hierarchical_control/actions/snap.h"


namespace mwoibn {
namespace hierarchical_control {
namespace actions {

/**
 * Replace an old task with a new task
 * ensures a continuity if the replacement happens at the stack end
 *
 */
class Replace : public Secondary {
public:
Replace(actions::Task& task_new, actions::Task& task_old, mwoibn::Matrix& P, mwoibn::VectorN& command, double mu, double dt, memory::Manager& memory, TaskMap& map) :
        Secondary(task_new, memory, map), _task_old(&task_old), _dt(dt), _mu(mu), _command(command){
        _q_new = command;
        _q_old = command;
        _setLimit();
}

Replace(mwoibn::Matrix& P, mwoibn::VectorN& command, double mu, double dt, memory::Manager& memory, TaskMap& map) :
        Secondary(memory, map), _dt(dt), _mu(mu), _command(command){
        _q_new = command;
        _q_old = command;
        _setLimit();
}

const Replace& operator=(const Replace& replace){
        return replace;
}


~Replace(){
}

void assign(actions::Task& task_new, actions::Task& task_old, actions::Snap& snap){
        // std::cout << "assign" << std::endl;
        _action = &task_new;
        _task_old = &task_old;
        _snap = &snap;
        _next = this;
}

void start(actions::Task& task_new, actions::Task& task_old, actions::Snap& snap){
        reset();
        assign(task_new, task_old, snap);
}

void start(actions::Task& task_new, actions::Task& task_old, actions::Snap& snap, double mu){
        _mu = mu;
        _setLimit();
        assign(task_new, task_old, snap);
}

void reset(){
        _t = 0;
        _p = 0;
}

virtual void run(){

        // std::cout << "run" << std::endl;
        _progress();

        _snap->run();

        _task_old->run();
        _q_old.noalias() = _command;

        _snap->restore();

        _action->run();
        _q_new.noalias() = _command;

        _command.noalias() = _q_new*_p;
        _command.noalias() += _q_old*(1-_p);
        //
        // std::cout << "old" << _task_old->baseAction().getTask().getError().transpose() << std::endl;
        // std::cout << "new" <<   _action->baseAction().getTask().getError().transpose() << std::endl;
}

virtual actions::Primary& secondAction(){
        return _task_old->baseAction();
}

virtual actions::Task& next() {
        _action = &(_action->next());
        _task_old = &(_task_old->next());
        if(_p < (1 - _eps)) {
                return *this;
        }
        else{
                release();
                return *_action;
        }
}

virtual void release(){
        // std::cout << "release" << std::endl;
        _memory.release(*_snap);
        _memory.release(*this);
        _snap = nullptr;
        _map[&_action->baseAction().getTask()] = _action;
        // _task_old->release();
}


protected:
double _p, _mu, _t, _dt, _eps = 1e-8;
actions::Task *_task_old; // should it be a compute action or a basic one?
actions::Snap* _snap;
mwoibn::VectorN _q_new, _q_old, &_command;

virtual void _progress(){
        _t += _dt;
        // _p = 1 - std::exp(-_mu*_t);
        _p = std::tanh(_mu*_t);
        std::cout << "t " << _t << "_p " << _p << ", eps " << _eps << "dt " << _dt << std::endl;
}

virtual void _setLimit(){
        std::cout << "_setLimit" << std::endl;
        _t = _dt;
        _progress();
        _eps = _p; // set end to the same step as the first one
        reset();
}
};

}
} // namespace package
} // namespace library
#endif
