#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_SECONDARY_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_SECONDARY_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/actions/task.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {

class Secondary : public Task {
public:
Secondary(actions::Task& action, memory::Manager& memory, TaskMap& map) : Task(memory), _action(&action), _map(map){
}
Secondary(memory::Manager& memory, TaskMap& map) : Task(memory), _map(map){
}

~Secondary(){
}

virtual void assign(actions::Task& action){
        _action = &action;
        _next   = &action;
}

virtual void release(){
        _next = _action;
        _map[&_action->baseAction().getTask()] = _action;
        _memory.release(*this);
}

virtual actions::Primary& baseAction() {
        return _action->baseAction();
}

virtual void run(){
}

virtual actions::Task& action(){
        return *_action;
}

virtual actions::Task& next() {
        _action = &(_action->next());
        if(_next != this) release();
        return *_next;
}

protected:
// Secondary(){
// }
actions::Task* _action;
//actions::Task* _next;
TaskMap& _map;
};

}
} // namespace package
} // namespace library
#endif
