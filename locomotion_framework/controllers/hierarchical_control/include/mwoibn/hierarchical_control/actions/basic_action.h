#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_BASIC_ACTION_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_BASIC_ACTION_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"

namespace mwoibn {
namespace hierarchical_control {
namespace actions {

class Compute;

class Basic {
public:
Basic(mwoibn::hierarchical_control::tasks::BasicTask& task) : _task(task){
}
~Basic(){
}

virtual void run() = 0;

mwoibn::hierarchical_control::tasks::BasicTask& getTask(){
        return _task;
}

virtual actions::Compute& baseAction() = 0;


protected:
mwoibn::hierarchical_control::tasks::BasicTask& _task;
};

}
} // namespace package
} // namespace library
#endif
