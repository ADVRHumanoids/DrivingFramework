#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_MERGE__MERGE_MANAGER_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_MERGE__MERGE_MANAGER_H

#include "mwoibn/hierarchical_control/memory/manager.h"
#include "mwoibn/hierarchical_control/memory/action_stack.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace actions {

namespace merge {

class End;
class Replace;
class Front;

class MergeManager : public memory::Manager {

public:
MergeManager(){
}

~MergeManager(){
}

using memory::Manager::release;
template<typename Action>
void release(std::vector<Action>& actions){
        for(auto& action : actions) release(action);
}

void release(Front& action){
        local_front.release(action);
}

void release(Replace& action){
        local_replace.release(action);
}

void release(End& action){
        local_end.release(action);
}

memory::ActionStack<Front> local_front;
memory::ActionStack<Replace> local_replace;
memory::ActionStack<End> local_end;

};

}
}
} // namespace package
} // namespace library
#endif
