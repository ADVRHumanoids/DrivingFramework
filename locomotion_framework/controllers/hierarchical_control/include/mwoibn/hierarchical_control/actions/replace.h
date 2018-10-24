#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_REPLACE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_REPLACE_H

#include "mwoibn/hierarchical_control/actions/basic.h"
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
Replace(actions::Task& task_new, actions::Task& task_old, mwoibn::Matrix& P, mwoibn::VectorN& command, memory::Manager& memory, maps::TaskMap& map) :
        Secondary(task_new, memory, map), _task_old(&task_old), _command(command){
        _snap = nullptr;
        _q_new = _command;
        _q_old = _command;
}

Replace(mwoibn::Matrix& P, mwoibn::VectorN& command, memory::Manager& memory, maps::TaskMap& map) :
        Secondary(memory, map), _command(command){
        _snap = nullptr;
        _q_new = _command;
        _q_old = _command;
}

const Replace& operator=(const Replace& replace){
        return replace;
}


virtual ~Replace(){
}

virtual void assign(actions::Task& task_new, actions::Task& task_old, actions::Snap& snap){
        _action = &task_new;
        _task_old = &task_old;

        if(_snap != nullptr && _snap != &snap)
          _memory.release(snap);

        _snap = &snap;
        _next = this;
}

virtual void start(actions::Task& task_new, actions::Task& task_old, actions::Snap& snap, double gain = 0){
        reset();
        assign(task_new, task_old, snap);
}

virtual void reset(){

}


virtual void run(){

        _snap->run();

        _task_old->run();
        _q_old.noalias() = _command;

        _snap->restore();

        _action->run();
        _q_new.noalias() = _command;

        _merge();

}

virtual actions::Task& secondAction(){
        return *_task_old;
}

virtual actions::Task& next() {
        _action = &(_action->next());
        _task_old = &(_task_old->next());
        if(!isDone()) {
                return *this;
        }
        else{
                release();
                return *_action;
        }
}

virtual void release(){
        _memory.release(*_snap);
        _memory.release(*this);
        _snap = nullptr;
        _map.swap(_action->baseAction().getTask(), *_action);
        // _task_old->release();
}

virtual bool isDone() = 0;

protected:
actions::Task *_task_old; // should it be a compute action or a basic one?
actions::Snap* _snap;
mwoibn::VectorN _q_new, _q_old, &_command;

virtual void _merge() = 0;

};

}
} // namespace package
} // namespace library
#endif
