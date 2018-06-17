#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_TASK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_TASK_H

#include "mwoibn/hierarchical_control/actions/basic_action.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {

class Primary;

class Task : public Basic {
public:
Task(memory::Manager& memory) : Basic(memory){
        _next = this;
}

~Task(){
}

virtual actions::Primary& baseAction() = 0;

virtual void swap(actions::Task& task){
        // std::cout << "swap" << std::endl;
        _next = &task;
        //_map[&(task->baseAction().getTask())] = &task;
}

virtual actions::Task& next(){
        actions::Task* ptr = _next;
        if(ptr != this) release();
        _next = this;

        return *ptr;
}

protected:
actions::Task* _next;

};

}
typedef std::map<tasks::BasicTask*, actions::Task* > TaskMap;

} // namespace package
} // namespace library
#endif
