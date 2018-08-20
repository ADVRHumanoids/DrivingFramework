#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_PRIMARY_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_PRIMARY_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/actions/task.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {

class Primary : public Task {
public:
Primary(tasks::BasicTask& task, memory::Manager& memory) : Task(memory), _task(task){
}
Primary(const Primary& other) : Task(other), _task(other._task){
}


virtual ~Primary(){
}

virtual actions::Primary& baseAction() {
        return *this;
}

virtual mwoibn::hierarchical_control::tasks::BasicTask& getTask(){
        return _task;
}

virtual bool updateGain(double gain){
        return false;
}

virtual bool updateGain(const mwoibn::VectorN& gain){
        return false;
}

protected:
mwoibn::hierarchical_control::tasks::BasicTask& _task;

};

}
} // namespace package
} // namespace library
#endif
